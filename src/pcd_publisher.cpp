#include <stdio.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/// Custom PCL point type with additional point data provided by the Robot Eye sensor
struct OcularPointType
{
  PCL_ADD_POINT4D;
  float amplitude;
  float reflectance;
  int pulseShapeDeviation;
  unsigned int timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (OcularPointType,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, amplitude, amplitude)
								   (float, reflectance, reflectance)
								   (float, pulseShapeDeviation, pulseShapeDeviation)
								   (unsigned int, timestamp, timestamp)
)

int main(int argc, char **argv)
{
    // setup ros for this node and get handle to ros system
    ros::init(argc, argv, "PCD file publisher");
    ros::start();

    // get private handle for this node to read parameters
    ros::NodeHandle privateH("~");

    ROS_INFO("Starting --[ PCD file publisher ]--");

    // get file names of cameras csv file and terrain obj file
    std::string pcdFileName, topicName;
    float rate;
    privateH.param<std::string>("pcd_file", pcdFileName, "scan.pcd");
    privateH.param<std::string>("topic_name", topicName, "pcd_file");
    privateH.param<float>("loop_rate", rate, 0.2);
    ROS_INFO("loading PCD file \"%s\"", pcdFileName.c_str());
    ros::Rate loopRate(rate);

    // load point cloud from pcd file
    pcl::PointCloud<OcularPointType> pointCloud;

    if (pcl::io::loadPCDFile<OcularPointType>(pcdFileName, pointCloud) == -1)
    {
    	ROS_INFO("Couldn't read point cloud file [%s] end of sequence reached", pcdFileName.c_str());
	ROS_INFO("Shutting down.");
    }
    else
    {
	ros::Publisher lidarScanPublisher = privateH.advertise<pcl::PointCloud<OcularPointType> >(topicName.c_str(),10);

	ROS_INFO("Publishing point cloud on topic \"%s\" once every 5 seconds.", topicName.c_str());

	while (ros::ok())
	{
	    lidarScanPublisher.publish(pointCloud.makeShared());

	    ros::spinOnce();
	    loopRate.sleep();
	}
    }

    return 1;
}
