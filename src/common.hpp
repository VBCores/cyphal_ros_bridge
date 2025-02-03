#pragma once

#include <chrono>
#include <stdint.h>

#include <cyphal/cyphal.h>

#define CID_R 2
#define CID_L 4
#define CID_PWR 9
#define CID_S 8

static constexpr CanardPortID ANGLE_PORT = 6998;
static constexpr CanardPortID ANGULAR_VELOCITY_PORT = 7006;
static constexpr CanardPortID TORQUE_PORT = 1423;
static constexpr CanardPortID VOLTAGE_PORT = 3423;

static inline uint64_t timeMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

#define EACH_N_TICKS(N, counter, code_blk) \
    if ((tick - (counter)) >= (N)) {        \
        (counter) = tick;                   \
        code_blk                            \
    }


class IMultipleListener {
public:
    virtual void add_listener(const std::string& topic, CanardNodeID source_node_id) = 0;
    virtual ~IMultipleListener() {}
};
