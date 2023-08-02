#pragma once
#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

enum flight_state {
    STATE_LAUNCHPAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_FREE_FALL,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
};

enum flight_event {
    EVENT_LAUNCH,
    EVENT_BURNOUT,
    EVENT_APOGEE,
    EVENT_PRIMARY_DEPLOY,
    EVENT_SECONDARY_DEPLOY,
    EVENT_LANDED
};

typedef struct {
    enum flight_state flight_state;
    float altitude;
    float ground_level;
    float velocity;
} fjalar_t;

extern fjalar_t fjalar_god;