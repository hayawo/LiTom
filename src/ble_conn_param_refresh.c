#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ホスト側が BLE の接続間隔を伸ばしたまま戻さないことがあるので、
// こちらから ZMK の希望値 (BT_PERIPHERAL_PREF_*) を要求し直す。
// Zephyr の自動更新 (BT_GAP_AUTO_UPDATE_CONN_PARAMS) は接続直後に 1 回
// 要求するだけで、その後にホストが間隔を伸ばしても何もしない。
// 間隔が伸びるとトラックボールのレポートが HID キューに溜まり、
// 動かしてからカーソルが遅れて動く / 途中が抜けて飛ぶ、という形で見える。

// ホストが要求を拒否して伸ばし直す場合に、要求を連打しないための最小間隔。
#define MIN_REQUEST_SPACING_MS 30000

static int64_t last_request_at;
static bool have_requested;

static void conn_param_refresh_work_handler(struct k_work *work) {
    if (!zmk_ble_active_profile_is_connected()) {
        return;
    }

    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn == NULL) {
        return;
    }

    struct bt_conn_info info;
    int err = bt_conn_get_info(conn, &info);
    if (err < 0 || info.type != BT_CONN_TYPE_LE) {
        bt_conn_unref(conn);
        return;
    }

    LOG_DBG("Host link: interval %d latency %d timeout %d", info.le.interval, info.le.latency,
            info.le.timeout);

    int64_t now = k_uptime_get();
    if (info.le.interval <= CONFIG_BT_PERIPHERAL_PREF_MAX_INT ||
        (have_requested && now - last_request_at < MIN_REQUEST_SPACING_MS)) {
        bt_conn_unref(conn);
        return;
    }

    have_requested = true;
    last_request_at = now;

    // Studio 接続中は latency を下げているので、現在値より上げない。
    uint16_t latency = MIN(info.le.latency, CONFIG_BT_PERIPHERAL_PREF_LATENCY);

    LOG_INF("Host link interval %d (x1.25ms) is slower than preferred; requesting %d-%d",
            info.le.interval, CONFIG_BT_PERIPHERAL_PREF_MIN_INT, CONFIG_BT_PERIPHERAL_PREF_MAX_INT);

    err = bt_conn_le_param_update(
        conn, BT_LE_CONN_PARAM(CONFIG_BT_PERIPHERAL_PREF_MIN_INT, CONFIG_BT_PERIPHERAL_PREF_MAX_INT,
                               latency, CONFIG_BT_PERIPHERAL_PREF_TIMEOUT));
    if (err < 0) {
        LOG_WRN("Failed to request connection parameter update (%d)", err);
    }

    bt_conn_unref(conn);
}

K_WORK_DELAYABLE_DEFINE(conn_param_refresh_work, conn_param_refresh_work_handler);

// ホストが接続間隔を変えたとき。直後に要求し返すと拒否されやすいので少し待つ。
static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency,
                             uint16_t timeout) {
    LOG_DBG("Connection parameters updated: interval %d latency %d timeout %d", interval, latency,
            timeout);

    if (interval > CONFIG_BT_PERIPHERAL_PREF_MAX_INT) {
        k_work_schedule(&conn_param_refresh_work, K_SECONDS(5));
    }
}

BT_CONN_CB_DEFINE(litom_conn_param_refresh_callbacks) = {
    .le_param_updated = le_param_updated,
};

// アイドルから復帰したとき (モニターのスリープ明けに最初に触ったときなど)。
// ホストが放置中に間隔を伸ばしていれば、ここで戻す。
static int conn_param_refresh_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev != NULL && ev->state == ZMK_ACTIVITY_ACTIVE) {
        have_requested = false;
        k_work_reschedule(&conn_param_refresh_work, K_NO_WAIT);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(litom_conn_param_refresh, conn_param_refresh_activity_listener);
ZMK_SUBSCRIPTION(litom_conn_param_refresh, zmk_activity_state_changed);
