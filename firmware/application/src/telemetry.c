#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>

#include "fjalar.h"

LOG_MODULE_REGISTER(TM, CONFIG_APP_LOG_LEVEL);

#if DT_ALIAS_EXISTS(data_flash)
void telemetry_flash_thread(fjalar_t *fjalar, void *p2, void *p3) {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(data_flash));

    if (!device_is_ready(flash_dev)) {
		LOG_ERR("Flash not ready");
		return 0;
	}
}
#endif
