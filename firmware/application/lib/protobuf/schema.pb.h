/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_SCHEMA_PB_H_INCLUDED
#define PB_SCHEMA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum flight_state {
    FLIGHT_STATE_LAUNCHPAD = 0,
    FLIGHT_STATE_BOOST = 1,
    FLIGHT_STATE_COAST = 2,
    FLIGHT_STATE_FREE_FALL = 3,
    FLIGHT_STATE_DROGUE_DESCENT = 4,
    FLIGHT_STATE_MAIN_DESCENT = 5,
    FLIGHT_STATE_LANDED = 6
} flight_state_t;

/* Struct definitions */
typedef struct acknowledge {
    bool success;
} acknowledge_t;

typedef struct pressure_reading {
    float pressure;
} pressure_reading_t;

typedef struct imu_reading {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} imu_reading_t;

typedef struct telemetry_packet {
    float altitude;
    float longitude;
    float latitude;
    int32_t pyro_bools;
} telemetry_packet_t;

typedef struct fjalar_data {
    pb_size_t which_data;
    union {
        acknowledge_t acknowledge;
        telemetry_packet_t telemetry_packet;
        imu_reading_t imu_reading;
        pressure_reading_t pressure_reading;
    } data;
} fjalar_data_t;

typedef struct fjalar_message {
    uint32_t time;
    int32_t sequence_number;
    bool has_data;
    fjalar_data_t data;
} fjalar_message_t;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _FLIGHT_STATE_MIN FLIGHT_STATE_LAUNCHPAD
#define _FLIGHT_STATE_MAX FLIGHT_STATE_LANDED
#define _FLIGHT_STATE_ARRAYSIZE ((flight_state_t)(FLIGHT_STATE_LANDED+1))








/* Initializer values for message structs */
#define ACKNOWLEDGE_INIT_DEFAULT                 {0}
#define PRESSURE_READING_INIT_DEFAULT            {0}
#define IMU_READING_INIT_DEFAULT                 {0, 0, 0, 0, 0, 0}
#define TELEMETRY_PACKET_INIT_DEFAULT            {0, 0, 0, 0}
#define FJALAR_DATA_INIT_DEFAULT                 {0, {ACKNOWLEDGE_INIT_DEFAULT}}
#define FJALAR_MESSAGE_INIT_DEFAULT              {0, 0, false, FJALAR_DATA_INIT_DEFAULT}
#define ACKNOWLEDGE_INIT_ZERO                    {0}
#define PRESSURE_READING_INIT_ZERO               {0}
#define IMU_READING_INIT_ZERO                    {0, 0, 0, 0, 0, 0}
#define TELEMETRY_PACKET_INIT_ZERO               {0, 0, 0, 0}
#define FJALAR_DATA_INIT_ZERO                    {0, {ACKNOWLEDGE_INIT_ZERO}}
#define FJALAR_MESSAGE_INIT_ZERO                 {0, 0, false, FJALAR_DATA_INIT_ZERO}

/* Field tags (for use in manual encoding/decoding) */
#define ACKNOWLEDGE_SUCCESS_TAG                  1
#define PRESSURE_READING_PRESSURE_TAG            1
#define IMU_READING_AX_TAG                       1
#define IMU_READING_AY_TAG                       2
#define IMU_READING_AZ_TAG                       3
#define IMU_READING_GX_TAG                       4
#define IMU_READING_GY_TAG                       5
#define IMU_READING_GZ_TAG                       6
#define TELEMETRY_PACKET_ALTITUDE_TAG            1
#define TELEMETRY_PACKET_LONGITUDE_TAG           2
#define TELEMETRY_PACKET_LATITUDE_TAG            3
#define TELEMETRY_PACKET_PYRO_BOOLS_TAG          4
#define FJALAR_DATA_ACKNOWLEDGE_TAG              1
#define FJALAR_DATA_TELEMETRY_PACKET_TAG         2
#define FJALAR_DATA_IMU_READING_TAG              3
#define FJALAR_DATA_PRESSURE_READING_TAG         4
#define FJALAR_MESSAGE_TIME_TAG                  1
#define FJALAR_MESSAGE_SEQUENCE_NUMBER_TAG       2
#define FJALAR_MESSAGE_DATA_TAG                  3

/* Struct field encoding specification for nanopb */
#define ACKNOWLEDGE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     success,           1)
#define ACKNOWLEDGE_CALLBACK NULL
#define ACKNOWLEDGE_DEFAULT NULL

#define PRESSURE_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    pressure,          1)
#define PRESSURE_READING_CALLBACK NULL
#define PRESSURE_READING_DEFAULT NULL

#define IMU_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    ax,                1) \
X(a, STATIC,   SINGULAR, FLOAT,    ay,                2) \
X(a, STATIC,   SINGULAR, FLOAT,    az,                3) \
X(a, STATIC,   SINGULAR, FLOAT,    gx,                4) \
X(a, STATIC,   SINGULAR, FLOAT,    gy,                5) \
X(a, STATIC,   SINGULAR, FLOAT,    gz,                6)
#define IMU_READING_CALLBACK NULL
#define IMU_READING_DEFAULT NULL

#define TELEMETRY_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    altitude,          1) \
X(a, STATIC,   SINGULAR, FLOAT,    longitude,         2) \
X(a, STATIC,   SINGULAR, FLOAT,    latitude,          3) \
X(a, STATIC,   SINGULAR, INT32,    pyro_bools,        4)
#define TELEMETRY_PACKET_CALLBACK NULL
#define TELEMETRY_PACKET_DEFAULT NULL

#define FJALAR_DATA_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,acknowledge,data.acknowledge),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,telemetry_packet,data.telemetry_packet),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,imu_reading,data.imu_reading),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,pressure_reading,data.pressure_reading),   4)
#define FJALAR_DATA_CALLBACK NULL
#define FJALAR_DATA_DEFAULT NULL
#define fjalar_data_t_data_acknowledge_MSGTYPE acknowledge_t
#define fjalar_data_t_data_telemetry_packet_MSGTYPE telemetry_packet_t
#define fjalar_data_t_data_imu_reading_MSGTYPE imu_reading_t
#define fjalar_data_t_data_pressure_reading_MSGTYPE pressure_reading_t

#define FJALAR_MESSAGE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FIXED32,  time,              1) \
X(a, STATIC,   SINGULAR, INT32,    sequence_number,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  data,              3)
#define FJALAR_MESSAGE_CALLBACK NULL
#define FJALAR_MESSAGE_DEFAULT NULL
#define fjalar_message_t_data_MSGTYPE fjalar_data_t

extern const pb_msgdesc_t acknowledge_t_msg;
extern const pb_msgdesc_t pressure_reading_t_msg;
extern const pb_msgdesc_t imu_reading_t_msg;
extern const pb_msgdesc_t telemetry_packet_t_msg;
extern const pb_msgdesc_t fjalar_data_t_msg;
extern const pb_msgdesc_t fjalar_message_t_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define ACKNOWLEDGE_FIELDS &acknowledge_t_msg
#define PRESSURE_READING_FIELDS &pressure_reading_t_msg
#define IMU_READING_FIELDS &imu_reading_t_msg
#define TELEMETRY_PACKET_FIELDS &telemetry_packet_t_msg
#define FJALAR_DATA_FIELDS &fjalar_data_t_msg
#define FJALAR_MESSAGE_FIELDS &fjalar_message_t_msg

/* Maximum encoded size of messages (where known) */
#define ACKNOWLEDGE_SIZE                         2
#define FJALAR_DATA_SIZE                         32
#define FJALAR_MESSAGE_SIZE                      50
#define IMU_READING_SIZE                         30
#define PRESSURE_READING_SIZE                    5
#define TELEMETRY_PACKET_SIZE                    26

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif