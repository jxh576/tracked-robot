/*
 * lms151_node.cpp
 *
 *  Created on: 24 Feb 2012
 *      Author: Jack Hargreaves
 */

#include <stdint.h>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include "lms151_node.h"
#include "lms151_cola.h"



lms151_node::lms151_node(const char* topic_name, size_t queue_size, size_t publish_rate,
            const char* host, size_t port, size_t expected_data_count)
        : driver(host, port, 0, expected_data_count),
          n(),
          loop_rate(publish_rate) {

    publisher = n.advertise<sensor_msgs::LaserScan> (topic_name, queue_size);
}

lms151_node::~lms151_node() {

}

inline bool lms151_node::GetData(sensor_msgs::LaserScan& data) {
    return driver.ReadMeasurement(data);
}

int main(int argv, char **argc) {

    ros::init(argv, argc, "laser_stub");

    const char host[] = "169.254.195.159";
    const int port = 2111;
    const size_t expected_data_count = 541;

    lms151_node node("laser_topic", 1000, 2, host, port, expected_data_count);

    // start driver reading from laser scanner
    boost::thread driver_thread(&lms100_cola::Run, &(node.driver));

    while (ros::ok()) {

        sensor_msgs::LaserScan data;

        if (node.GetData(data)) {
            if (!data.ranges.empty()) {
                ROS_DEBUG("publishing -- data count %i", data.ranges.size());
            }
            node.publisher.publish(data);
        }
        else {
            ROS_DEBUG("no data to publish");
        }

        ros::spinOnce();
        node.loop_rate.sleep();
    }

    ROS_INFO("waiting for driver for thread to shutdown");
    node.driver.Shutdown();
    driver_thread.join();

    return 0;
}
