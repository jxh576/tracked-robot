/*
 * lms151_node.cpp
 *
 *  Created on: 24 Feb 2012
 *      Author: Jack Hargreaves
 */

#include <stdint.h>
#include <signal.h>

#include <boost/thread.hpp>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include "lms151_node.h"
#include "lms151_cola.h"



lms151_node::lms151_node(const char* topic_name, size_t queue_size, size_t publish_rate) :
          n(),
          loop_rate(publish_rate) {

    publisher = n.advertise<sensor_msgs::LaserScan> (topic_name, queue_size);
}

lms151_node::~lms151_node() {

}

//inline bool lms151_node::GetData(sensor_msgs::LaserScan& data) {
//    return driver.ReadMeasurement(data);
//}



struct sigint_handler_data {
    lms100_cola* driver;
    boost::thread* driver_thread;
};

static struct sigint_handler_data sigint_handler_data;
static struct sigaction sigIntHandler;

void sigint_handler(int s) {
    sigint_handler_data.driver->Shutdown();
    sigint_handler_data.driver_thread->join();
    exit(1);
}

void setup_sigint_handler(lms100_cola* driver, boost::thread* driver_thread) {
    sigint_handler_data.driver = driver;
    sigint_handler_data.driver_thread = driver_thread;

    sigIntHandler.sa_handler = sigint_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argv, char **argc) {

    ros::init(argv, argc, "laser_stub");

    const char host[] = "169.254.195.159";
    const int port = 2111;
    const size_t expected_data_count = 541;

    lms151_node node("laser_topic", 1000, 2);

    lms100_cola driver(host, port, 0, expected_data_count);
    // start driver reading from laser scanner
    boost::thread driver_thread(&lms100_cola::Run, &driver);

    setup_sigint_handler(&driver, &driver_thread);

    while (ros::ok()) {

        sensor_msgs::LaserScan data;

        if (driver.ReadMeasurement(data)) {
            if (!data.ranges.empty()) {
                ROS_INFO("publishing scan %i from time %i",data.header.seq, data.header.stamp.sec);
                ROS_INFO("start angle %f, end angle %f, increments %f", data.angle_min, data.angle_max, data.angle_increment);
            }
            node.publisher.publish(data);
        }
        else {
            ROS_INFO("no data to publish");
        }

        ros::spinOnce();
        node.loop_rate.sleep();
    }

    ROS_INFO("waiting for driver for thread to shutdown");
    driver.Shutdown();
    driver_thread.join();

    return 0;
}
