#include <functional>
#include <tuple>

#include <cyphal/cyphal.h>

#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <std_msgs/Float32.h>

TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)

namespace CyphalBridge {

template <class A, class B>
using Translator = std::tuple<std::function<A(const B&)>, std::function<B(const A&)>>;

const Translator<Real32::Type, std_msgs::Float32> float_translator {
    [](const std_msgs::Float32& ros_msg) {
        auto cyphal_msg = Real32::Type();
        cyphal_msg.value = ros_msg.data;
        return cyphal_msg;
    },
    [](const Real32::Type& cyphal_msg) {
        auto ros_msg = std_msgs::Float32();
        ros_msg.data = cyphal_msg.value;
        return ros_msg;
    }
};


}
