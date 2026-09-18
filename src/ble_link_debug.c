#include <zephyr/bluetooth/conn.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// BLE リンクの実際の状態を 5 秒ごとにログへ出す診断用。
// 知りたいのは次の 2 点:
//   - ホストが実際に使っている接続間隔 (7.5ms の要求が通っているか)
//   - キーボードが送ろうとしている入力の件数 (PC 側の実測と突き合わせる)
// マウスレポートがキューからあふれた場合は、ZMK 本体の hog.c が
// "Consumer message queue full" を WRN で出す (マウス用の経路だが
// メッセージ文言がキーボードの消費者キー用の流用になっている)。

#define REPORT_INTERVAL K_SECONDS(5)

static atomic_t rel_events;
static atomic_t sync_events;

static void link_debug_input_listener(struct input_event *ev) {
    if (ev->type == INPUT_EV_REL) {
        atomic_inc(&rel_events);
    }

    if (ev->sync) {
        atomic_inc(&sync_events);
    }
}

INPUT_CALLBACK_DEFINE(NULL, link_debug_input_listener);

static void link_debug_work_handler(struct k_work *work) {
    uint32_t rel = (uint32_t)atomic_set(&rel_events, 0);
    uint32_t sync = (uint32_t)atomic_set(&sync_events, 0);

    if (!zmk_ble_active_profile_is_connected()) {
        LOG_INF("link: not connected | input rel %u sync %u per 5s", rel, sync);
        goto reschedule;
    }

    struct bt_conn *conn = zmk_ble_active_profile_conn();
    if (conn == NULL) {
        LOG_INF("link: no conn | input rel %u sync %u per 5s", rel, sync);
        goto reschedule;
    }

    struct bt_conn_info info;
    int err = bt_conn_get_info(conn, &info);
    if (err == 0 && info.type == BT_CONN_TYPE_LE) {
        // 接続間隔の単位は 1.25ms。小数第 1 位まで出す。
        uint32_t ms10 = info.le.interval * 125 / 10;
        LOG_INF("link: interval %u (%u.%u ms) latency %u timeout %u | input rel %u sync %u per 5s",
                info.le.interval, ms10 / 10, ms10 % 10, info.le.latency, info.le.timeout, rel,
                sync);
    } else {
        LOG_WRN("link: bt_conn_get_info failed (%d)", err);
    }

    bt_conn_unref(conn);

reschedule:
    k_work_reschedule(k_work_delayable_from_work(work), REPORT_INTERVAL);
}

K_WORK_DELAYABLE_DEFINE(link_debug_work, link_debug_work_handler);

static int link_debug_init(void) {
    k_work_reschedule(&link_debug_work, REPORT_INTERVAL);
    return 0;
}

SYS_INIT(link_debug_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
