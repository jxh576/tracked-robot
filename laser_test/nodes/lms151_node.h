/*
 * lms151_node.h
 *
 *  Created on: 24 Feb 2012
 *      Author: Jack Hargreaves
 */

#ifndef LMS151_NODE_H_
#define LMS151_NODE_H_

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>

#include "lms151_cola.h"

class lms151_node {

public:

    lms151_node(const char* topic_name, size_t queue_size, size_t publish_rate);
//            const char* host, size_t port, size_t expected_data_count);
    ~lms151_node();

//    bool GetData(sensor_msgs::LaserScan& data);

//    lms100_cola driver;
    ros::NodeHandle n;
    ros::Rate loop_rate;
    ros::Publisher publisher;


};

#endif /* LMS151_NODE_H_ */
