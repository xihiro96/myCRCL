//
// Created by mhendrix8 on 7/25/16.
//

#ifndef MY_CRCL_SERVERCONFIGURATION_H
#define MY_CRCL_SERVERCONFIGURATION_H

#include <ros/ros.h>

namespace crcl
{
struct RobotConfiguration
{
	std::vector<std::string> joint_names;
	std::map<std::string, size_t> joint_name_to_index;
	std::set<std::string> gripper_joint_names;
	std::vector<double> joint_multipliers;
	std::vector<double> joint_offsets;
	std::vector<std::pair<double,double>> joint_bounds;
};
struct CommConfiguration
{
	std::string address;
	int port;
	int frequency;
};

void getConfig(ros::NodeHandle pn, RobotConfiguration& config);
void getConfig(ros::NodeHandle pn, CommConfiguration& config);

}

#endif //MY_CRCL_SERVERCONFIGURATION_H
