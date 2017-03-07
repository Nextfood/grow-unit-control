#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <sstream>
#include "InterfaceSerialConnection.h"
#include "Tsl2591Sensor.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    InterfaceSerialConnection isc("/dev/ttyUSB0");
    Tsl2591Sensor sensor(isc);

    std::string uniqueId = sensor.getInfo("unique_id");

    ros::Publisher light_data_full_pub = n.advertise<std_msgs::Float64>(std::string("/light_data/TSL2591/full"), 1000);
    ros::Publisher light_data_ir_pub = n.advertise<std_msgs::Float64>(std::string("/light_data/TSL2591/ir"), 1000);
    ros::Publisher light_data_lux_pub = n.advertise<std_msgs::Float64>(std::string("/light_data/TSL2591/lux"), 1000);

    ros::Rate loop_rate(1);

    while (ros::ok()) {
        sensor.update();
        std_msgs::Float64 full,ir,lux;
        full.data = sensor.getFull();
        ir.data = sensor.getIr();
        lux.data = sensor.getLux();

        light_data_full_pub.publish(full);
        light_data_ir_pub.publish(ir);
        light_data_lux_pub.publish(lux);

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
