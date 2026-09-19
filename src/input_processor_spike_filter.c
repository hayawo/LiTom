#define DT_DRV_COMPAT litom_input_processor_spike_filter

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/input_processor.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// センサーが省電力から復帰した直後、PMW3610 が壊れた移動量を返すことが
// ある。実測では 3 分 20 秒ぶりに触った瞬間から 165ms の間に 11 件、
// 符号がばらばらで最大 1575 という値が届き、カーソルが画面端まで飛んだ。
// 600cpi / 16ms 間隔では、通常の移動は 20〜80、速く弾いても 200 程度。

struct spike_filter_config {
    uint16_t type;
    size_t codes_len;
    const uint16_t *codes;
    int32_t max_delta;
    int32_t suppress_ms;
};

struct spike_filter_data {
    int64_t suppress_until;
};

static bool is_tracked_code(const struct spike_filter_config *cfg, uint16_t code) {
    for (size_t i = 0; i < cfg->codes_len; i++) {
        if (cfg->codes[i] == code) {
            return true;
        }
    }

    return false;
}

static int spike_filter_handle_event(const struct device *dev, struct input_event *event,
                                     uint32_t param1, uint32_t param2,
                                     struct zmk_input_processor_state *state) {
    const struct spike_filter_config *cfg = dev->config;
    struct spike_filter_data *data = dev->data;

    if (event->type != cfg->type || !is_tracked_code(cfg, event->code)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int32_t value = event->value;
    int32_t magnitude = value < 0 ? -value : value;
    int64_t now = k_uptime_get();

    if (magnitude > cfg->max_delta) {
        LOG_WRN("Dropping implausible delta %d on code %d", value, event->code);
        data->suppress_until = now + cfg->suppress_ms;
        event->value = 0;
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // 一度崩れると数件続くので、直後のしばらくはまとめて捨てる。
    if (data->suppress_until > now) {
        LOG_DBG("Dropping %d on code %d (within suppress window)", value, event->code);
        event->value = 0;
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api spike_filter_driver_api = {
    .handle_event = spike_filter_handle_event,
};

#define SPIKE_FILTER_INST(n)                                                                       \
    static const uint16_t spike_filter_codes_##n[] = DT_INST_PROP(n, codes);                       \
    static const struct spike_filter_config spike_filter_config_##n = {                            \
        .type = DT_INST_PROP(n, type),                                                             \
        .codes_len = DT_INST_PROP_LEN(n, codes),                                                   \
        .codes = spike_filter_codes_##n,                                                           \
        .max_delta = DT_INST_PROP(n, max_delta),                                                   \
        .suppress_ms = DT_INST_PROP(n, suppress_ms),                                               \
    };                                                                                             \
    static struct spike_filter_data spike_filter_data_##n = {};                                    \
    DEVICE_DT_INST_DEFINE(n, NULL, NULL, &spike_filter_data_##n, &spike_filter_config_##n,         \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                        \
                          &spike_filter_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SPIKE_FILTER_INST)
