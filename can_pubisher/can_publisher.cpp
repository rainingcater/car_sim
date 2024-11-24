#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "can_publisher_node");
    ros::NodeHandle nh;

    // 创建一个发布者，发布类型为can_msgs::Frame的消息到"can_topic" topic上
    ros::Publisher publisher = nh.advertise<can_msgs::Frame>("can_topic", 10);

    // 创建SocketCAN套接字
    int can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        ROS_ERROR_STREAM("Failed to open CAN socket");
        return -1;
    }

    // 设置CAN设备名
    struct ifreq ifr;
    std::string can_device_name = "can0"; // 假设CAN设备名为can0
    strcpy(ifr.ifr_name, can_device_name.c_str());
    ioctl(can_socket, SIOCGIFINDEX, &ifr);

    // 绑定CAN设备到套接字
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        ROS_ERROR_STREAM("Failed to bind CAN socket to device: " << can_device_name);
        close(can_socket);
        return -1;
    }

    while (ros::ok()) {
        // 从CAN卡中读取数据
        struct can_frame frame;
        int bytes_read = read(can_socket, &frame, sizeof(struct can_frame));

        if (bytes_read < 0) {
            ROS_ERROR_STREAM("Error reading from CAN socket");
            continue;
        }

        // 创建一个can_msgs::Frame消息对象，并设置数据
        can_msgs::Frame msg;
        msg.id = frame.can_id;
        msg.dlc = frame.can_dlc;
        for (int i = 0; i < frame.can_dlc; ++i) {
            msg.data[i] = frame.data[i];
        }

        // 发布消息
        publisher.publish(msg);
    }

    // 关闭套接字
    close(can_socket);