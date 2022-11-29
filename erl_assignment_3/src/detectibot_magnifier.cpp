/** @package erl_assignment_3
*
*	@file detectibot_magnifier.cpp
*	@brief This node implements the Detection task with Aruco on a single camera
*
*	@author Iacopo Pietrasanta
*	@version 1.0.0
*
*	Subscribes to: <BR>
*		/clock 											[rosgraph_msgs/Clock]
*		/robot/camera1/image_raw 						[sensor_msgs/Image]
*	Publishes to: <BR>
*		/rosout [rosgraph_msgs/Log]
*
*
*	Services: <BR>
* 		/aruco_markers
* 		/detectibot_magnifier/get_loggers
* 		/detectibot_magnifier/set_logger_level
*
*	Client Services: <BR>
		None
*
*	Action Services: <BR>
*    	None
*
*	Description: <BR>
*		This node handles the Aruco's marker's detection task. After the Aruco
* 	marker is detected is sent through the output topic 
*
*/


#include "ros/ros.h"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "sensor_msgs/Image.h"
#include "erl_assignment_3_msgs/GetArucoIds.h"

#include <string>
#include <vector>
#include <algorithm>
#include <unordered_set>

class detectibot_magnifier_class
{
public:

	/** detectibot_magnifier_class class constructor */
	detectibot_magnifier_class() :
		nh_("~"), it_(nh_), useCamInfo_(true)
	{
		// image input
		image_sub_ = it_.subscribe(in_topic, 1, &detectibot_magnifier_class::image_callback, this);

		// service
		get_ids_service = nh_.advertiseService("/aruco_markers", &detectibot_magnifier_class::get_ids, this);

		// setup ArUco
		nh_.param<bool>("use_camera_info", useCamInfo_, false);
		camParam_ = aruco::CameraParameters();
	}

	/** receive the image, get the ARUCO markers, then send the detections through the out topic */
	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		ros::Time curr_stamp = msg->header.stamp;
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			// copy the image
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			inImage_ = cv_ptr->image;

			// collect the markers
			markers_.clear();
			mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

			if(markers_.size() > 0)
			{
				int prev_size = ids.size();
				for(aruco::Marker& m : markers_)
					ids.insert(m.id);

				if(prev_size != ids.size())
					ROS_INFO_STREAM("found markers : " << set2string(ids));
			}
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}

	/** get the IDs identified by aruco */
	bool get_ids(erl_assignment_3_msgs::GetArucoIds::Request  &req, erl_assignment_3_msgs::GetArucoIds::Response &res)
	{
		res.isEmpty = ids.empty();
		res.ids = set2vector(ids);

		return true;
	}

private:

	/// class node handle
	ros::NodeHandle nh_;

	/// arUco detector
	aruco::MarkerDetector mDetector_;

	/// array of markers from the callback
	std::vector<aruco::Marker> markers_;

	/// arUco camera parameters
	aruco::CameraParameters camParam_;

	/// size of each marker
	double marker_size_;

	/// ???
	bool useCamInfo_;

	/// image transport object
	image_transport::ImageTransport it_;

	/// last received image
	cv::Mat inImage_;

	/// input img topic name
	std::string in_topic = "/robot/camera1/image_raw";

	/// input img subscriber
	image_transport::Subscriber image_sub_;

	/// service /get_ids
	ros::ServiceServer get_ids_service;

	/// the set of the found IDs
	std::unordered_set<int> ids;

	/** convert a set into a string */
	std::string set2string(std::unordered_set<int>& st)
	{
		std::string str = "";

		for(auto& elem : st)
			str += "" + std::to_string(elem) + " ";

		return str;
	}

	/** convert a set into a dynamic array */
	std::vector<int> set2vector(std::unordered_set<int>& st)
	{
		std::vector<int> v;

		for(auto& elem : st) v.push_back(elem);

		return v;
	}
};


/** main function */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "detectibot_magnifier");
	ros::AsyncSpinner spinner(3);
	spinner.start();

	detectibot_magnifier_class echo;
	ros::waitForShutdown();

	return 0;
}
