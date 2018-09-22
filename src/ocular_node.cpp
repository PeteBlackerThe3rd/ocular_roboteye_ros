/*-----------------------------------------------------------\\
||                                                           ||
||                 Ocular LIDAR ROS Driver                   ||
||               ----------------------------                ||
||                                                           ||
||    Surrey Space Centre - STAR lab                         ||
||    (c) University of 2017                                 ||
||    Pete dot Blacker at Gmail dot com                      ||
||                                                           ||
\\-----------------------------------------------------------//

ocular_node.cpp

A basic implementation of a ROS driver for the Ocular LIDAR
using the provided UDP protocol specification.

-------------------------------------------------------------*/
#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/asio.hpp"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

// custom services
#include "ocular_lidar_driver/StartFullFieldScan.h"
#include "ocular_lidar_driver/StartBoundedElevationScan.h"
#include "ocular_lidar_driver/StartRegionScan.h"
#include "ocular_lidar_driver/StopScan.h"
#include "ocular_lidar_driver/HomeSensor.h"

using namespace boost::asio;
using namespace boost::chrono;

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

int nothing;

/*! \brief Main Robot Eye driver class encapsulating the UDP communication, ros services and topics.
 *
 * This object handles all UDP communication with the Robot Eye sensors, publishes received point
 * cloud data and registers and handles the service calls used to control the sensor.
 *
 */
class ocularDriver
{
public:
	ocularDriver(boost::asio::io_service& io_service,
				 std::string topicName,
				 std::string frameId,
				 int packetsToMessageRatio = 1)
			: laserDataSocket(io_service, ip::udp::endpoint(ip::udp::v4(), 4371))
	{
		// start listening for laser data
		startReceivingLaserData();

		// set default ocular port and IP address
		sensorIPAddress = "";

		this->packetsToMessageRatio = packetsToMessageRatio;
		packetsInCloud = 0;

		// setup point cloud publisher
		ros::NodeHandle n;
		cloudPublisher = n.advertise<pcl::PointCloud<OcularPointType> >(topicName,10);
		pointCloud.header.frame_id = frameId;
	}

	/// Structure used to hold the details of a Robot Eye sensor detected by the findEyes method.
	struct eyeInfo
	{
		std::string ipAddress;
		std::string serialNumber;
	};

	/// Method to identify all Robot Eye sensors connected to the network
	/*
	 * Method to identify all Robot Eye sensors connected to the network
	 *
	 * Returns:
	 * std::vector<eyeInfo> vector containing the ip address and serial number of all sensors found.
	 *
	 */
	std::vector<eyeInfo> findEyes()
	{
		eyeList.clear();
		io_service io_service;
		boost::chrono::duration<float> waitDuration(1.0);  // one second wait for replies

		// open broadcast socket on port 4365 for the Robot Eye search
		ip::udp::socket receiveSocket(io_service);
		receiveSocket.open(ip::udp::v4());
		receiveSocket.set_option(ip::udp::socket::reuse_address(true));
		receiveSocket.set_option(socket_base::broadcast(true));

		receiveSocket.async_receive_from(boost::asio::buffer(findEyesBuffer), findEyesRemoteEndpoint,
									     boost::bind(&ocularDriver::findEyesResponse, this,
												     boost::asio::placeholders::error,
												     boost::asio::placeholders::bytes_transferred));

		// open broadcast socket to send request to all Robot Eye sensors
		ip::udp::endpoint remote_endpoint = ip::udp::endpoint(ip::address_v4::broadcast(), ocularPort);
		receiveSocket.send_to(buffer("RECQRE", 6), remote_endpoint);


		// wait for require time processing incoming datagrams every millisecond
		steady_clock::time_point start = steady_clock::now();
		while (steady_clock::now() < start + waitDuration)
		{
			io_service.poll();
			boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
		}

		receiveSocket.close();

		return eyeList;
	}

	char findEyesBuffer[1024];
	ip::udp::endpoint findEyesRemoteEndpoint;
	std::vector<eyeInfo> eyeList;

	void findEyesResponse(const boost::system::error_code& error, std::size_t bytesTransferred)
	{
		if (!error)
		{
			// check that this is a Seek Robot Eyes reponse message (ERCQRE)
			if (strncmp(findEyesBuffer, "ERCQRE", 6) == 0)
			{
				eyeInfo newEye;
				newEye.ipAddress = findEyesRemoteEndpoint.address().to_string();
				newEye.serialNumber = std::string(&findEyesBuffer[6], bytesTransferred-6);
				eyeList.push_back(newEye);

				ROS_INFO("Found Robot Eye [%s] on ip address [%s]",
						 newEye.serialNumber.c_str(),
						 newEye.ipAddress.c_str());
			}
		}
	}

	void setIPAddress(std::string newIPAddress)
	{
		sensorIPAddress = newIPAddress;
	}

	bool home()
	{
		return (sendRequest("REHOME") >= 0);
	}

	bool stopScanning()
	{
		int laserResponse = sendRequest("RESRBE");	// stop laser
		int scanResponse = sendRequest("RESTOP");	// stop the scanning motion

		return (laserResponse >= 0 && scanResponse >= 0);
	}

	/// Method to start a full FOV scan
	/*
	 *  Method to start a full FOV scan
	 *
	 *  Parameters
	 *  ------------
	 *  float speed : the azimuth speed of the scan in revolutions per second
	 *  float lines : The number of vertical lines to scan between +35 and -35 degrees elevation
	 *  [Note lines can be a non-integer value which will result in interpolated values]
	 *
	 */
	bool startFullScan(float speed = 5.0, float lines = 100.0)
	{
		char data[14];
		strncpy(data, "RESFFS", 6);

		unsigned int speedValue = speed * 65536 + 1;
		unsigned int linesValue = lines * 65536 + 1;

		data[6] = (speedValue >> 24) & 0xff;
		data[7] = (speedValue >> 16) & 0xff;
		data[8] = (speedValue >> 8 ) & 0xff;
		data[9] = (   speedValue   ) & 0xff;

		data[10] = (linesValue >> 24) & 0xff;
		data[11] = (linesValue >> 16) & 0xff;
		data[12] = (linesValue >> 8 ) & 0xff;
		data[13] = (   linesValue   ) & 0xff;

		return (sendRequest(data, sizeof(data)) >= 0 );
	}

	/// Method to start a bounded elevation scan
	/*
	 *  Method to start a full FOV scan
	 *
	 *  Parameters
	 *  ------------
	 *  float speed : the azimuth speed of the scan in revolutions per second
	 *  float eleMin: the minimum elevation in degrees from horizontal
	 *  float eleMax: the maximum elevation in degrees from horizontal
	 *  float lines : The number of vertical lines to scan between +35 and -35 degrees elevation
	 *  [Note lines can be a non-integer value which will result in interpolated values]
	 *
	 */
	bool startBoundedElevationScan(float speed = 5.0,
			                       float elMin = -10,
								   float elMax = 10,
								   float lines = 100.0)
	{
		char data[22];
		strncpy(data, "RESBES", 6);

		unsigned int speedValue = speed * 65536;
		unsigned int eleMinValue = elMin * 4294967296/360.0;
		unsigned int eleMaxValue = elMax * 4294967296/360.0;
		unsigned int linesValue = lines * 65536;

		data[6] = (speedValue >> 24) & 0xff;
		data[7] = (speedValue >> 16) & 0xff;
		data[8] = (speedValue >> 8 ) & 0xff;
		data[9] = (   speedValue   ) & 0xff;

		data[10] = (eleMinValue >> 24) & 0xff;
		data[11] = (eleMinValue >> 16) & 0xff;
		data[12] = (eleMinValue >> 8 ) & 0xff;
		data[13] = (   eleMinValue   ) & 0xff;

		data[14] = (eleMaxValue >> 24) & 0xff;
		data[15] = (eleMaxValue >> 16) & 0xff;
		data[16] = (eleMaxValue >> 8 ) & 0xff;
		data[17] = (   eleMaxValue   ) & 0xff;

		data[18] = (linesValue >> 24) & 0xff;
		data[19] = (linesValue >> 16) & 0xff;
		data[20] = (linesValue >> 8 ) & 0xff;
		data[21] = (   linesValue   ) & 0xff;

		return (sendRequest(data, sizeof(data)) >= 0 );
	}

        bool startRegionScan(float azMin = -5,
                             float azMax = -5,
                             float elMin = -10,
                             float elMax = 10,
                             float speed = 5.0,
                             float lines = 100.0)
        {
                char data[30];
                strncpy(data, "RESRES", 6);

                unsigned int speedValue = speed * 65536;
                unsigned int eleMinValue = elMin * 4294967296/360.0;
                unsigned int eleMaxValue = elMax * 4294967296/360.0;
                unsigned int azMinValue = azMin * 4294967296/360.0;
                unsigned int azMaxValue = azMax * 4294967296/360.0;
                unsigned int linesValue = lines * 65536;

                data[6] = (azMinValue >> 24) & 0xff;
                data[7] = (azMinValue >> 16) & 0xff;
                data[8] = (azMinValue >> 8 ) & 0xff;
                data[9] = (   azMinValue   ) & 0xff;

                data[10] = (azMaxValue >> 24) & 0xff;
                data[11] = (azMaxValue >> 16) & 0xff;
                data[12] = (azMaxValue >> 8 ) & 0xff;
                data[13] = (   azMaxValue   ) & 0xff;

                data[14] = (eleMinValue >> 24) & 0xff;
                data[15] = (eleMinValue >> 16) & 0xff;
                data[16] = (eleMinValue >> 8 ) & 0xff;
                data[17] = (   eleMinValue   ) & 0xff;

                data[18] = (eleMaxValue >> 24) & 0xff;
                data[19] = (eleMaxValue >> 16) & 0xff;
                data[20] = (eleMaxValue >> 8 ) & 0xff;
                data[21] = (   eleMaxValue   ) & 0xff;

                data[22] = (speedValue >> 24) & 0xff;
                data[23] = (speedValue >> 16) & 0xff;
                data[24] = (speedValue >> 8 ) & 0xff;
                data[25] = (   speedValue   ) & 0xff;

                data[26] = (linesValue >> 24) & 0xff;
                data[27] = (linesValue >> 16) & 0xff;
                data[28] = (linesValue >> 8 ) & 0xff;
                data[29] = (   linesValue   ) & 0xff;

                return (sendRequest(data, sizeof(data)) >= 0 );
        }

	static const int modeHighPenetration = 0x00;
	static const int modeFast = 0x01;
	static const int modeHighSpeed = 0x02;

	static const int multiEchoModeAll = 0x00;
	static const int multiEchoModeFirst = 0x01;
	static const int multiEchoModeLast = 0x02;
	static const int multiEchoModeHighestAmplitude = 0x03;
	static const int multiEchoModeHighestReflectance = 0x04;

	// start the laser scanning
	bool startRangeBearingElevationSensor(int mode = modeHighPenetration,
										  int multiEchoMode = multiEchoModeAll)
	{
		// verify that mode and multi echo mode are valid
		if (mode < 0 || mode > 0x02)
		{
			ROS_ERROR("Error, invalid mode (0x%02X) given to startRangeBearingElevationSensor function.",
					  mode);
			return false;
		}
		if (multiEchoMode < 0 || multiEchoMode > 0x04)
		{
			ROS_ERROR("Error, invalid multiEchoMode (0x%02X) given to startRangeBearingElevationSensor function.",
					  multiEchoMode);
			return false;
		}

		char data[21];
		strncpy(data, "RERRBE", 6);

		// option 0x00
		int receivePort = 4371; // need to mod this so an automatically detected free port number is given

		// set option 1
		data[6] = 0x01;
		data[7] = 0x00;
		data[8] = 0x00;
		data[9] = 0x00;
		data[10] = mode;

		// set option 2
		data[11] = 0x02;
		data[12] = 0x00;
		data[13] = 0x00;
		data[14] = 0x00;
		data[15] = multiEchoMode;

		// set option 0
		data[16] = 0x00;
		data[17] = 0x00;
		data[18] = 0x00;
		data[19] = (receivePort >> 8) & 0xff;
		data[20] = (  receivePort   ) & 0xff;

		return (sendRequest(data, sizeof(data)) >= 0 );
	}

	void startServiceHandlers()
	{
		static ros::ServiceServer stopScanService;
		static ros::ServiceServer homeSensorService;
		static ros::ServiceServer startFullFieldScanService;
                static ros::ServiceServer startBoundedElevationScanService;
                static ros::ServiceServer startRegionScanService;

		ros::NodeHandle n;

		stopScanService = n.advertiseService("ocular_lidar_node/StopScan",
											 &ocularDriver::stopScanCallback,
											 this);

		homeSensorService = n.advertiseService("ocular_lidar_node/HomeSensorService",
											 &ocularDriver::homeSensorCallback,
											 this);

		startFullFieldScanService = n.advertiseService("ocular_lidar_node/StartFullFieldScan",
											 &ocularDriver::startFullFieldScanCallback,
											 this);
                startBoundedElevationScanService = n.advertiseService("ocular_lidar_node/StartBoundedElevationScan",
                                                                                         &ocularDriver::startBoundedElevationScanCallback,
                                                                                         this);
                startRegionScanService = n.advertiseService("ocular_lidar_node/StartRegionScan",
                                                                                         &ocularDriver::startRegionScanCallback,
                                                                                         this);
	}

	// method to handle StopScan service request
	bool stopScanCallback(ocular_lidar_driver::StopScan::Request &req,
						  ocular_lidar_driver::StopScan::Response &resp)
	{
		ROS_INFO("Stop scan request received");

		if (stopScanning())
		{
			resp.receivedOk = true;
			resp.errorMsg = "";
		}
		else
		{
			resp.receivedOk = false;
			resp.errorMsg = "Failed to correctly send stop scanning request to the sensor.";
		}
		return true;
	}

	// method to handle HomeSensor service request
	bool homeSensorCallback(ocular_lidar_driver::HomeSensor::Request &req,
						    ocular_lidar_driver::HomeSensor::Response &resp)
	{
		ROS_INFO("Home sensor request received");

		if (home())
		{
			resp.receivedOk = true;
			resp.errorMsg = "";
		}
		else
		{
			resp.receivedOk = false;
			resp.errorMsg = "Failed to correctly send home request to the sensor.";
		}
		return true;
	}

	// method to handle StartFullFieldScan service request
	bool startFullFieldScanCallback(ocular_lidar_driver::StartFullFieldScan::Request &req,
						  	  	    ocular_lidar_driver::StartFullFieldScan::Response &resp)
	{
		ROS_INFO("StartFullFieldScan scan request received");

		if (!startFullScan(req.rotationsPerSecond,
						   req.numberOfLines))
		{
			resp.startedOk = false;
			resp.errorMsg = "Error failed to start scanning motion";
		}
		else if (!startRangeBearingElevationSensor(req.mode,
												   req.multiEchoMode))
		{
			resp.startedOk = false;
			resp.errorMsg = "Error failed to start laser ranging";
		}
		else
		{
			resp.startedOk = true;
			resp.errorMsg = "";
		}
		return true;
	}

        bool startBoundedElevationScanCallback(ocular_lidar_driver::StartBoundedElevationScan::Request &req,
                                                                    ocular_lidar_driver::StartBoundedElevationScan::Response &resp)
        {
                ROS_INFO("StartBoundedElevationScan scan request received");

                if (!startBoundedElevationScan(req.rotationsPerSecond,
                                   req.eleMin,
                                   req.eleMax,
                                   req.numberOfLines))
                {
                        resp.startedOk = false;
                        resp.errorMsg = "Error failed to start scanning motion";
                }
                else if (!startRangeBearingElevationSensor(req.mode,
                                                                                                   req.multiEchoMode))
                {
                        resp.startedOk = false;
                        resp.errorMsg = "Error failed to start laser ranging";
                }
                else
                {
                        resp.startedOk = true;
                        resp.errorMsg = "";
                }
                return true;
        }

        bool startRegionScanCallback(ocular_lidar_driver::StartRegionScan::Request &req,
                                                                    ocular_lidar_driver::StartRegionScan::Response &resp)
        {
                ROS_INFO("StartRegionScan scan request received");

                if (!startRegionScan(req.azMax,
                                   req.azMin,
                                   req.eleMin,
                                   req.eleMax,
                                   req.rotationsPerSecond,
                                   req.numberOfLines))
                {
                        resp.startedOk = false;
                        resp.errorMsg = "Error failed to start scanning motion";
                }
                else if (!startRangeBearingElevationSensor(req.mode,
                                                                                                   req.multiEchoMode))
                {
                        resp.startedOk = false;
                        resp.errorMsg = "Error failed to start laser ranging";
                }
                else
                {
                        resp.startedOk = true;
                        resp.errorMsg = "";
                }
                return true;
        }


private:

	/// Method to send a basic opcode request which doesn't include any payload data
	int sendRequest(std::string opcode)
	{
		char data[6];
		strncpy(data, opcode.c_str(), 6);

		return sendRequest(data, sizeof(data));
	}

	/// Method to identify all Robot Eye sensors connected to the network
	/*
	 * Method to identify all Robot Eye sensors connected to the network
	 *
	 * Returns:
	 * std::vector<eyeInfo> vector containing the ip address and serial number of all sensors found.
	 *
	 */
	int sendRequest(char *data, int dataLen)
	{
		if (sensorIPAddress.length() == 0)
		{
			ROS_ERROR("Attempt to send a robot eye request with no IP address set.");
			return -1;
		}

		io_service io_service;
		boost::chrono::duration<float> waitDuration(5.0);  // one second wait for replies

		// open broadcast socket on port 4365 for the Robot Eye search
		ip::udp::socket receiveSocket(io_service);
		receiveSocket.open(ip::udp::v4());

		receiveSocket.async_receive_from(boost::asio::buffer(requestResponseBuffer), requestRemoteEndpoint,
									     boost::bind(&ocularDriver::sendRequestResponse, this,
												     boost::asio::placeholders::error,
												     boost::asio::placeholders::bytes_transferred));

		// open broadcast socket to send request to all Robot Eye sensors
		requestRemoteEndpoint = ip::udp::endpoint(ip::address::from_string(sensorIPAddress), ocularPort);

		// compose expected response code
		requestResponseExpected[0] = 'E';
		requestResponseExpected[1] = 'A';
		strncpy(&requestResponseExpected[2], &data[2], 4);

		// create buffer for command packet
		std::vector<char> commandPacket;
		for (int d=0; d<dataLen; ++d)
			commandPacket.push_back(data[d]);

		receiveSocket.send_to(buffer(commandPacket), requestRemoteEndpoint);
		requestResponseReceived = false;
		correctResponseReceived = false;

		// wait for require time processing incoming datagrams every millisecond
		steady_clock::time_point start = steady_clock::now();
		while (steady_clock::now() < start + waitDuration && requestResponseReceived == false)
		{
			io_service.poll();
			boost::this_thread::sleep_for(boost::chrono::milliseconds(10));
		}

		receiveSocket.close();

		// if no response was recieved from the sensor
		if (requestResponseReceived == false)
		{
			char response[7];
			strncpy(response, data, 6);
			response[6] = 0;
			ROS_ERROR("Error Timeout waiting for response from \"%s\" command.", response);
			return -1;
		}

		if (correctResponseReceived)
			return 6;
		else
			return -1;
	}

	char requestResponseBuffer[1024];
	bool requestResponseReceived;
	bool correctResponseReceived;
	char requestResponseExpected[6];
	ip::udp::endpoint requestRemoteEndpoint;

	void sendRequestResponse(const boost::system::error_code& error, std::size_t bytesTransferred)
	{
		if (!error)
		{
			requestResponseReceived = true;

			// if this is an error message response
			if (requestResponseBuffer[0] == 'E' && requestResponseBuffer[1] == 'E')
			{
				if (requestResponseBuffer[6] != 0x00) // if this is not a 'no error' erro!
				{
					if (bytesTransferred == 7)
					{
						ROS_ERROR("Error received from [%c%c%c%c] command : (0x%02X) %s.",
								requestResponseBuffer[2], requestResponseBuffer[3],
								requestResponseBuffer[4], requestResponseBuffer[5],
								requestResponseBuffer[6],
								getErrorMsg(requestResponseBuffer[6]).c_str());
					}
					else
						ROS_ERROR("Malformed error message received from Ocular LIDAR.");
				}
			}

			// if this is the expected response
			else if (strncmp(requestResponseBuffer, requestResponseExpected, 6) == 0)
			{
				correctResponseReceived = true;
			}

			else // if an unexpected response was received
			{
				char response[7];
				strncpy(response, requestResponseBuffer, 6);
				response[6] = 0;
				ROS_ERROR("Unexpected response [%s] received from Robot Eye sensor.", response);
			}
		}
	}

	/// Function to convert a Robot Eye error code into a human readable string
	std::string getErrorMsg(unsigned char error)
	{
		switch(error)
		{
			case 0x00: return "No Error";
			case 0x01: return "Invalid Argument Length";
			case 0x02: return "Argument Out Of Range";
			case 0x03: return "Error Not Ready";
			case 0x04: return "Error Not Homed";
			case 0x05: return "Error Invalid Argument";
			case 0x06: return "Error Unknown Command";
			case 0x07: return "Error Unsupported Command";
			case 0x08: return "Error Scan too Sparse";
			case 0x09: return "Error Busy";
			case 0x0a: return "Error Bad Flash Page";
			case 0x0b: return "Error Bad Flash Key";
			case 0x0c: return "Error Stabilisation Running";
			case 0xff: return "Other Error";
		}
	}

	void startReceivingLaserData()
	{
		laserDataSocket.async_receive_from(boost::asio::buffer(receiveBuffer), remoteEndpoint,
								   boost::bind(&ocularDriver::receiveLaserData, this,
											   boost::asio::placeholders::error,
											   boost::asio::placeholders::bytes_transferred));
	}

	void receiveLaserData(const boost::system::error_code& error, std::size_t bytesTransferred)
	{
		if (!error)
		{
			// test if this is a Eye Broadcast Range Bearing Elevation Packet
			if (bytesTransferred >= 6 && strncmp((char*)receiveBuffer, "EBRBEP", 6) == 0)
			{
				//pointCloud.points.clear();

				int start = 6;
				int pointCount = 0;
				OcularPointType newPoint;

				while (start + pointCount*22 < bytesTransferred)
				{
					// extract this point from the data
					int offset = start + pointCount*22;
					++pointCount;

					unsigned int timeStampRaw;
					unsigned int elevationRaw, azimuthRaw, depthRaw, amplitudeRaw, reflectanceRaw;
					double timeStamp;
					float elevation, azimuth; // angles in Radians
					float depth, amplitude, reflectance;
					int PSD; // pulse shape deviation

					// decode the depth first because if it's zero we don't bother with the rest
					depthRaw = receiveBuffer[offset+12] << 24 |
							   receiveBuffer[offset+13] << 16 |
							   receiveBuffer[offset+14] << 8 |
							   receiveBuffer[offset+15];
					depth = depthRaw / 1000.0;					// convert mm into m

					if (depth != 0.0)
					{
						timeStampRaw = receiveBuffer[offset] << 24 |
									   receiveBuffer[offset+1] << 16 |
									   receiveBuffer[offset+2] << 8 |
									   receiveBuffer[offset+3];
						timeStamp = timeStampRaw / 10000000.0; // convert 100's of nano seconds into seconds.

						azimuthRaw = receiveBuffer[offset+4] << 24 |
									 receiveBuffer[offset+5] << 16 |
									 receiveBuffer[offset+6] << 8 |
									 receiveBuffer[offset+7];
						azimuth = azimuthRaw * ((2*M_PI) / 4294967296);	// convert ocular angle encoding into radians

						elevationRaw = receiveBuffer[offset+8 ] << 24 |
									   receiveBuffer[offset+9 ] << 16 |
									   receiveBuffer[offset+10] << 8 |
									   receiveBuffer[offset+11];
						elevation = elevationRaw * ((2*M_PI) / 4294967296);	// convert ocular angle encoding into radians

						amplitudeRaw = receiveBuffer[offset+16] << 8 |
									   receiveBuffer[offset+17];
						newPoint.amplitude = amplitudeRaw / 100.0;			// convert 0.01 dB into dB

						reflectanceRaw = receiveBuffer[offset+18] << 8 |
										 receiveBuffer[offset+19];
						newPoint.reflectance = reflectanceRaw / 100.0;			// convert 0.01 dB into dB

						newPoint.pulseShapeDeviation = receiveBuffer[offset+20] << 8 |
													   receiveBuffer[offset+21];

						// convert spherical coordinates to Cartesian coordinates
						newPoint.x = sin(azimuth) * cos(elevation) * depth;
						newPoint.y = cos(azimuth) * cos(elevation) * depth;
						newPoint.z = sin(elevation) * depth;

						pointCloud.points.push_back(newPoint);
					}
				}

				// if the number of packets worth of point cloud data has been reached then
				// publish this point cloud
				++packetsInCloud;
				if (packetsInCloud >= packetsToMessageRatio)
				{
					cloudPublisher.publish(pointCloud.makeShared());
					pointCloud.points.clear();
				}

				
			}

			// re attach the receive handler to listen for more laser datagrams
			startReceivingLaserData();
		}
	}

	/// Factory set value of the Robot Eye sensors listing port
	static const int ocularPort = 4365;

	/// IP address of the current sensor in use stored as a string in '.' separated decimal format
	std::string sensorIPAddress;

	/// Socket object used to listen for incoming laser datagrams
	ip::udp::socket laserDataSocket;
	ip::udp::endpoint remoteEndpoint;

	/// Buffer used to store incoming laser datagrams
	unsigned char receiveBuffer[2500];
	int packetsToMessageRatio;
	int packetsInCloud;

	/// The ROS publisher object used to publish the point clouds of each datagram
	ros::Publisher cloudPublisher;

	/// The point cloud object used to collect and publish the points as they are received from the sensor
	pcl::PointCloud<OcularPointType> pointCloud;
};

int main(int argc, char **argv)
{
	ROS_INFO("--[ Occular Robotics LIDAR Driver Node ]--");

	// setup ros for this node
	ros::init(argc, argv, "occular_node");
	ros::start();

	// get private node handle;
	ros::NodeHandle nh("~");

	/*std::vector<std::string> keys;
	if (nh.getParamNames(keys))
	{
		for (int i=0; i<keys.size(); ++i)
			printf("[%d] - \"%s\"\n", i, keys[i].c_str());
	}
	else
		ROS_ERROR("Failed to get parameter keys");*/

	// process parameters and check for basic validity
	std::string detectionMethod;
	nh.param<std::string>("detection_method", detectionMethod, "auto");

	if (detectionMethod != "auto" && detectionMethod != "ip_address" && detectionMethod != "serial_number")
	{
		ROS_ERROR("detection method [%s] is invalid, must be one of \"auto\", \"ip_address\" or \"serial_number\"",
				  detectionMethod.c_str());
		exit(1);
	}

	std::string detectionLabel;
	if (!nh.getParam("detection_label", detectionLabel) && detectionMethod != "auto")
	{
		ROS_ERROR("detection_label must be specified if not using the 'auto' detection method.");
		exit(1);
	}

	std::string sensorFrameId;
	nh.param<std::string>("sensor_frame_id", sensorFrameId, "robot_eye_frame");

	std::string pointCloudTopic;
	nh.param<std::string>("point_cloud_topic", pointCloudTopic, "robot_eye_points");

	// catch io exceptions during UDP communications
	try
	{
		// create driver object used to access the sensor
		io_service io_service;
		ocularDriver server(io_service, pointCloudTopic, sensorFrameId);

		// find list of Robot Eye sensors attached to the network
		std::vector<ocularDriver::eyeInfo> eyeList = server.findEyes();

		if (eyeList.empty())
		{
			ROS_ERROR("Didn't detect any Robot Eye sensors, check your network configuration.");
			exit(1);
		}

		// depending on the detection method chosen process this list to find the correct sensor
		int selectedSensor = -1;
		if (detectionMethod == "auto")
		{
			if (eyeList.size() > 1)
				ROS_WARN("Auto-detect mode enabled but more than one sensor has been found. Selection will be largely random!");
			selectedSensor = 0;
		}
		else if (detectionMethod == "ip_address")
		{
			for (int i=0; i<eyeList.size(); ++i)
				if (eyeList[i].ipAddress == detectionLabel)
					selectedSensor = i;
			if (selectedSensor == -1)
				ROS_ERROR("Sensor with requested IP address of \"%s\" not found on network.", detectionLabel.c_str());
		}
		else if (detectionMethod == "serial_number")
		{
			for (int i=0; i<eyeList.size(); ++i)
				if (eyeList[i].serialNumber == detectionLabel)
					selectedSensor = i;
			if (selectedSensor == -1)
				ROS_ERROR("Sensor with requested serial number of \"%s\" not found on network.", detectionLabel.c_str());
		}
		if (selectedSensor == -1)
		{
			ROS_ERROR("Found the following Robot Eye sensors on the network:");
			for (int i=0; i<eyeList.size(); ++i)
				ROS_ERROR("Robot Eye [%s] on ip address %s",
						  eyeList[i].serialNumber.c_str(),
						  eyeList[i].ipAddress.c_str());
			exit(1);
		}
		server.setIPAddress(eyeList[selectedSensor].ipAddress);

		if (server.home())
		{
			ROS_INFO("Successfully connected to Robot Eye \"%s\" on IP address %s",
					 eyeList[selectedSensor].serialNumber.c_str(),
					 eyeList[selectedSensor].ipAddress.c_str());
			ROS_INFO("Point cloud data will be published on topic \"%s\" with frame_id \"%s\"",
					 pointCloudTopic.c_str(),
					 sensorFrameId.c_str());
		}
		else
		{
			ROS_ERROR("Communication timeout attempting to home Robot Eye sensor.");
			exit(1);
		}

		ROS_INFO("Listening for service requests.");
		server.startServiceHandlers();

		// temporary test bit
		//server.startFullScan();
		//server.startRangeBearingElevationSensor(ocularDriver::modeHighSpeed,
		//										ocularDriver::multiEchoModeHighestAmplitude);

	    ros::Rate loopRate(1000);
		while(ros::ok())
		{
			// Handle UDP events
			io_service.poll();

			// Handle ROS events
			ros::spinOnce();

			// pause for the required delay
			loopRate.sleep();
		}

		// to be on the safe side stop the sensor when closing the node
		server.stopScanning();
	}
	catch (std::exception& e)
	{
		ROS_ERROR("UDP Error [%s]", e.what());
	}

	ROS_INFO("--[ Node shutdown OK ]--");
	fflush(stdout);
	return 0;
}
