//
// Created by mhendrix8 on 7/25/16.
//
#include "CRCLSocketInterface.h"
#include "ServerConfiguration.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

crcl::RobotConfiguration rConfig;
crcl::CommConfiguration cConfig;

ros::Publisher ft_pub;
ros::Publisher joint_pub;
ros::Publisher state_pub;
gtri::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;

void setupCRCL(crcl::CRCLSocketInterface& crclSocket)
{
	CrclInitCanon initCanon;
	crclSocket.send(&initCanon);
	crclSocket.receive(250);

	sleep(1);

	CrclSetAngleUnits setAngle;
	setAngle.setRadian();
	crclSocket.send(&setAngle);
	crclSocket.receive(250);

	sleep(1);

	CrclSetLengthUnits setLength;
	setLength.setMeter();
	crclSocket.send(&setLength);
	crclSocket.receive(250);

	sleep(1);
}

void shutdownCRCL(crcl::CRCLSocketInterface& crclSocket)
{
	CrclEndCanon endCanon;
	crclSocket.send(&endCanon);
}

void publishStatus(gtri::shared_ptr<CRCLStatusFile>& latestStatus)
{
//	CRCLStatusFile* latestStatus;
	if (gtri::shared_ptr<CRCLStatusFile>() == latestStatus)
	{
		return;
	}
	ros::Time stamp = ros::Time::now();

	// Read the end-effector position
	tf::Vector3 xAxis(latestStatus->CRCLStatus->PoseStatus->Pose->XAxis->I->val,
	                  latestStatus->CRCLStatus->PoseStatus->Pose->XAxis->J->val,
	                  latestStatus->CRCLStatus->PoseStatus->Pose->XAxis->K->val);
	tf::Vector3 zAxis(latestStatus->CRCLStatus->PoseStatus->Pose->ZAxis->I->val,
	                  latestStatus->CRCLStatus->PoseStatus->Pose->ZAxis->J->val,
	                  latestStatus->CRCLStatus->PoseStatus->Pose->ZAxis->K->val);
	tf::Vector3 yAxis = zAxis.cross(xAxis);

	assert(fabs(xAxis.length2() - 1.0) < 1e-6);
	assert(fabs(yAxis.length2() - 1.0) < 1e-6);
	assert(fabs(zAxis.length2() - 1.0) < 1e-6);

	tf::Matrix3x3 R(xAxis.x(), yAxis.x(), zAxis.x(),
                    xAxis.y(), yAxis.y(), zAxis.y(),
                    xAxis.z(), yAxis.z(), zAxis.z());

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(
	                        latestStatus->CRCLStatus->PoseStatus->Pose->Point->X->val,
	                        latestStatus->CRCLStatus->PoseStatus->Pose->Point->Y->val,
	                        latestStatus->CRCLStatus->PoseStatus->Pose->Point->Z->val));

	tf::Quaternion q;
	R.getRotation(q);
	transform.setRotation(q);
	tf_broadcaster->sendTransform(tf::StampedTransform(transform, stamp, "base", "end_effector"));

	// Read the force/torque sensor
	static bool isFirstStatus = true;
	if (latestStatus->CRCLStatus->PoseStatus->Wrench)
	{
		geometry_msgs::WrenchStamped wrench;
		wrench.header.frame_id = "/ft_sensor";
		wrench.header.stamp = stamp;
		wrench.wrench.force.x = latestStatus->CRCLStatus->PoseStatus->Wrench->Force->I->val;
		wrench.wrench.force.y = latestStatus->CRCLStatus->PoseStatus->Wrench->Force->J->val;
		wrench.wrench.force.z = latestStatus->CRCLStatus->PoseStatus->Wrench->Force->K->val;
		wrench.wrench.torque.x = latestStatus->CRCLStatus->PoseStatus->Wrench->Moment->I->val;
		wrench.wrench.torque.y = latestStatus->CRCLStatus->PoseStatus->Wrench->Moment->J->val;
		wrench.wrench.torque.z = latestStatus->CRCLStatus->PoseStatus->Wrench->Moment->K->val;
		ft_pub.publish(wrench);
	}
	else if (isFirstStatus)
	{
		isFirstStatus = false;
		ROS_WARN_ONCE("Pose status does not contain an end-effector wrench.");
	}

	// Read the joint positions
	sensor_msgs::JointState js;
	js.header.frame_id = "/base_link";
	js.header.stamp = stamp;
	for (JointStatusType* status : *latestStatus->CRCLStatus->JointStatuses->JointStatus)
	{
		int j = status->JointNumber->val - 1;
		js.name.push_back(rConfig.joint_names[j]);
		js.position.push_back((M_PI/180.0 * status->JointPosition->val) * rConfig.joint_multipliers[j] + rConfig.joint_offsets[j]);
	}

	ParallelGripperStatusType* gripperStatus = dynamic_cast<ParallelGripperStatusType*>(latestStatus->CRCLStatus->GripperStatus);
	if (nullptr != gripperStatus)
	{
		int j = rConfig.joint_name_to_index["finger_joint"];
		js.name.push_back("finger_joint");
		// NB: Joint is in units of "separation", not degrees like the other joints
		js.position.push_back((gripperStatus->Separation->val) * rConfig.joint_multipliers[j] + rConfig.joint_offsets[j]);
	}

	joint_pub.publish(js);

	// Read the execution state
	std_msgs::String sMsg;
	sMsg.data = latestStatus->CRCLStatus->CommandStatus->CommandState->val;
	state_pub.publish(sMsg);
}

/**
 * @brief updateStatus sends a 'getstatus' request to the server, parses the response, and publishes the status information to the appropriate topics.
 */
void updateStatus(gtri::shared_ptr<crcl::CRCLSocketInterface>& crclSocket)
{
	gtri::shared_ptr<CRCLStatusFile> latestStatus;

	// Flush the buffer
	crclSocket->receive(5);

	CrclGetStatus getStat;
	crclSocket->send(&getStat);
	latestStatus = crclSocket->receive();
	if (gtri::shared_ptr<CRCLStatusFile>() == latestStatus)
	{
		crclSocket = gtri::make_shared<crcl::CRCLSocketInterface>(cConfig.address.c_str(), cConfig.port);
		return;
	}

	ROS_DEBUG_STREAM("Publishing Status...");
	publishStatus(latestStatus);
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "crcl_status_monitor");
	ros::NodeHandle n, pn("~");

	crcl::getConfig(pn, rConfig);
	crcl::getConfig(pn, cConfig);

	if(cConfig.address.empty() || cConfig.port <= 0)
	{
		std::cerr << "Please provide a valid address and port for the robot server." << std::endl;
		return 1;
	}

	ft_pub = n.advertise<geometry_msgs::WrenchStamped>("/forceTorque", 10);
	joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
	state_pub = n.advertise<std_msgs::String>("/execution_state", 1);
	tf_broadcaster = gtri::make_shared<tf::TransformBroadcaster>();

	gtri::shared_ptr<crcl::CRCLSocketInterface> crclSocket = gtri::make_shared<crcl::CRCLSocketInterface>(cConfig.address.c_str(), cConfig.port);
	setupCRCL(*crclSocket);

	ros::Rate rate(cConfig.frequency);
	while(ros::ok())
	{
		updateStatus(crclSocket);
		ros::spinOnce();
		rate.sleep();
	}

	shutdownCRCL(*crclSocket);


	return 0;
}
