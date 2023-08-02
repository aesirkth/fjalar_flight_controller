#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <pla.h>

#include "fjalar.h"
#include "sensors.h"
#include "filter.h"

LOG_MODULE_REGISTER(flight, CONFIG_APP_LOG_LEVEL);

#define FLIGHT_THREAD_PRIORITY 7
#define FLIGHT_THREAD_STACK_SIZE 1024

#define ALTITUDE_PRIMARY_WINDOW_SIZE 5
#define ALTITUDE_SECONDARY_WINDOW_SIZE 40

#define IMU_PRIMARY_WINDOW_SIZE 5

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1);

K_THREAD_STACK_DEFINE(flight_thread_stack, FLIGHT_THREAD_STACK_SIZE);
struct k_thread flight_thread_data;
k_tid_t flight_thread_id;

void init_flight_state(fjalar_t *fjalar) {
    flight_thread_id = k_thread_create(
		&flight_thread_data,
		flight_thread_stack,
		K_THREAD_STACK_SIZEOF(flight_thread_stack),
		(k_thread_entry_t) flight_state_thread,
		fjalar, NULL, NULL,
		FLIGHT_THREAD_PRIORITY, 0, K_NO_WAIT
	);
}

static float pressure_to_altitude(float pressure) {
    return pow(1013.5 / (double) pressure * 10.0 - 1.0, 1.0 / 5.257) * (20.0 + 273.15) / 0.0065;
}

void flight_state_thread(fjalar_t *fjalar, void *p2, void *p1) {
    struct k_poll_event events[2] = {
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &pressure_msgq),
        K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &imu_msgq),
    };

    altitude_filter_t altitude_filter;
    altitude_filter_init(&altitude_filter);
    struct pressure_queue_entry pressure;
    struct imu_queue_entry imu;

    k_poll(&events[0], 1, K_FOREVER);
    k_poll(&events[1], 1, K_FOREVER);
    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;
    if (k_msgq_get(&imu_msgq, &imu, K_NO_WAIT) != 0) {
        LOG_ERR("Could not get initial imu value");
    }
    if (k_msgq_get(&pressure_msgq, &pressure, K_NO_WAIT) != 0) {
        LOG_ERR("Could not get initial pressure value");
    }

    while (true) {
        if (k_poll(events, 2, K_MSEC(1000))) {
            LOG_ERR("Stopped receiving measurements");
        }

        if (k_msgq_get(&pressure_msgq, &pressure, K_NO_WAIT) == 0) {
            events[0].state = K_POLL_STATE_NOT_READY;
            float raw_altitude = pressure_to_altitude(pressure.pressure);
            altitude_filter_update(&altitude_filter, raw_altitude, pressure.t);
            fjalar->altitude = altitude_filter_get_altitude(&altitude_filter);
            fjalar->velocity = altitude_filter_get_velocity(&altitude_filter);
            if (fjalar->flight_state == STATE_LAUNCHPAD) {
                fjalar->ground_level = fjalar->altitude;
            }
            LOG_INF("estimated altitude: %f", fjalar->altitude - fjalar->ground_level);
            LOG_INF("estimated velocity: %f", fjalar->velocity);

            if (fjalar->velocity < 0 && fjalar->flight_state != STATE_LAUNCHPAD) {
                LOG_INF("Apogee!");
            }
        }
        if (k_msgq_get(&imu_msgq, &imu, K_NO_WAIT) == 0) {
            events[1].state = K_POLL_STATE_NOT_READY;
            vec3 vec = {imu.ax, imu.ay, imu.az};
            if (vec3_norm(vec) > 12.0) {
                fjalar->flight_state = STATE_BOOST;
            }
        }
    }
}