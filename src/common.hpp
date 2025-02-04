#pragma once

#include <chrono>
#include <stdint.h>

#include <cyphal/cyphal.h>

static inline uint64_t timeMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

class IMultipleListener {
public:
    virtual void add_listener(const std::string& topic, CanardNodeID source_node_id) = 0;
    virtual ~IMultipleListener() {}
};
