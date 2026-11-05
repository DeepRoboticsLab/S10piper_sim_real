#pragma once
// In-process ROS API shim, ONLY for building the unchanged real S10/Piper adapter.
// It has no ROS/DDS implementation or service client; all sockets are loopback.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>

namespace drdds { namespace msg {
struct Joint {
    float position=0, velocity=0, torque=0, kp=0, kd=0;
    uint16_t status_word=1, control_word=0;
};
struct JointsValue { std::array<Joint,16> joints_data; };
struct JointsData { using SharedPtr=std::shared_ptr<JointsData>; JointsValue data; };
struct JointsDataCmd { using SharedPtr=std::shared_ptr<JointsDataCmd>; JointsValue data; };
struct ImuValue {
    float roll=0,pitch=0,yaw=0,acc_x=0,acc_y=0,acc_z=0,omega_x=0,omega_y=0,omega_z=0;
};
struct ImuData { using SharedPtr=std::shared_ptr<ImuData>; ImuValue data; };
struct BatteryData { using SharedPtr=std::shared_ptr<BatteryData>; };
}}

namespace sim_dds {
inline constexpr std::array<float,16> direction{
    1,1,-1,1, 1,-1,1,-1, -1,1,-1,1, -1,-1,1,-1};
inline constexpr std::array<float,16> offset_degrees{
    -35,-145,156,0, 35,-145,156,0, -35,145,-156,0, 35,145,-156,0};
inline float offset(int i) { return offset_degrees[i]*0.017453292519943295f; }
inline int port(const char* variable,int fallback) {
    const char* value=std::getenv(variable);
    int result=value?std::stoi(value):fallback;
    if(result<1024 || result>65535)throw std::runtime_error("invalid local simulation port");
    return result;
}
inline sockaddr_in address(int port) {
    sockaddr_in a{};a.sin_family=AF_INET;a.sin_port=htons(port);
    a.sin_addr.s_addr=htonl(INADDR_LOOPBACK);return a;
}
}

namespace rclcpp {
inline std::atomic<bool> alive{true};
inline bool ok() { return alive; }
inline void shutdown() { alive=false; }
struct SensorDataQoS {};
template<class T> class Subscription {
public: using SharedPtr=std::shared_ptr<Subscription<T>>;
};
template<class T> class Publisher {
    int fd_=-1;
    sockaddr_in target_{};
public:
    using SharedPtr=std::shared_ptr<Publisher<T>>;
    Publisher() {
        fd_=::socket(AF_INET,SOCK_DGRAM,0);
        if(fd_<0)throw std::runtime_error("simulation publisher socket failed");
        target_=sim_dds::address(sim_dds::port("S10_SIM_COMMAND_PORT",11882));
    }
    ~Publisher(){if(fd_>=0)::close(fd_);}
    size_t get_subscription_count()const{return 1;}
    void publish(const T& msg) {
        static_assert(std::is_same_v<T,drdds::msg::JointsDataCmd>);
        // [kp16, q16, kd16, dq16, tau16, control_word16]. The adapter uses
        // hardware coordinates; invert those back to MuJoCo policy axes.
        std::array<float,96> wire{};
        for(int i=0;i<16;++i) {
            const auto& j=msg.data.joints_data[i];
            wire[i]=j.kp;
            wire[16+i]=j.position*sim_dds::direction[i]+sim_dds::offset(i);
            wire[32+i]=j.kd;
            wire[48+i]=j.velocity*sim_dds::direction[i];
            wire[64+i]=j.torque*sim_dds::direction[i];
            wire[80+i]=j.control_word;
        }
        if(::sendto(fd_,wire.data(),sizeof(wire),0,
                    reinterpret_cast<sockaddr*>(&target_),sizeof(target_))<0)
            throw std::runtime_error("local simulation command send failed");
    }
};

class Node {
    int fd_=-1;
    std::function<void(drdds::msg::JointsData::SharedPtr)> joint_;
    std::function<void(drdds::msg::ImuData::SharedPtr)> imu_;
public:
    using SharedPtr=std::shared_ptr<Node>;
    explicit Node(const std::string&) {
        fd_=::socket(AF_INET,SOCK_DGRAM,0);
        const auto a=sim_dds::address(sim_dds::port("S10_SIM_STATE_PORT",11880));
        if(fd_<0 || ::bind(fd_,reinterpret_cast<const sockaddr*>(&a),sizeof(a))<0)
            throw std::runtime_error("local simulation state port busy");
    }
    ~Node(){if(fd_>=0)::close(fd_);}
    template<class T> typename Publisher<T>::SharedPtr create_publisher(const std::string&,int) {
        return std::make_shared<Publisher<T>>();
    }
    template<class T,class Q,class F>
    typename Subscription<T>::SharedPtr create_subscription(const std::string&,Q,F f) {
        if constexpr(std::is_same_v<T,drdds::msg::JointsData>)joint_=f;
        if constexpr(std::is_same_v<T,drdds::msg::ImuData>)imu_=f;
        return std::make_shared<Subscription<T>>();
    }
    void Poll() {
        fd_set set;FD_ZERO(&set);FD_SET(fd_,&set);timeval timeout{0,10000};
        if(::select(fd_+1,&set,nullptr,nullptr,&timeout)<=0)return;
        std::array<float,76> p{};
        if(::recv(fd_,p.data(),sizeof(p),0)!=sizeof(p))return;
        auto joint=std::make_shared<drdds::msg::JointsData>();
        auto imu=std::make_shared<drdds::msg::ImuData>();
        imu->data={p[1],p[2],p[3],p[4],p[5],p[6],p[7],p[8],p[9]};
        for(int i=0;i<16;++i) {
            auto& j=joint->data.joints_data[i];
            j.position=(p[10+i]-sim_dds::offset(i))*sim_dds::direction[i];
            j.velocity=p[32+i]*sim_dds::direction[i];
            j.torque=p[54+i]*sim_dds::direction[i];j.status_word=1;
        }
        if(joint_)joint_(joint);
        if(imu_)imu_(imu);
    }
};
inline void spin(const Node::SharedPtr& node){while(ok())node->Poll();}
}
