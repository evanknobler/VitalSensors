#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <stdint.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "drivers.h"
#include "wireless.h"

void main(void)
{
    int err;
    uint32_t red_sample, ir_sample;

    err = max86140_init();
    if (err) {
        LOG_ERR("MAX86140 init failed");
        return;
    }

    err = max30001_init();
    if (err) {
        LOG_ERR("MAX30001 init failed");
        return;
    }

    err = init_wireless();
    if (err) return;
}
