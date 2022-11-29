/** @package erl_assignment_3
*
*	@file img_echo.cpp
*	@brief This node reads the camera input and show it onto a floating window
*
*	@author Iacopo Pietrasanta
*	@version 1.0.0
*
*	Subscribes to: <BR>
*		/clock 											[rosgraph_msgs/Clock]
*		/robot/camera1/image_raw 						[sensor_msgs/Image]
*	Publishes to: <BR>
* 		/img_echo [sensor_msgs/Image]
* 		/img_echo/compressed [sensor_msgs/CompressedImage]
*		/img_echo/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
*		/img_echo/compressed/parameter_updates [dynamic_reconfigure/Config]
* 		/img_echo/compressedDepth [sensor_msgs/CompressedImage]
* 		/img_echo/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* 		/img_echo/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
* 		/img_echo/theora [theora_image_transport/Packet]
* 		/img_echo/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* 		/img_echo/theora/parameter_updates [dynamic_reconfigure/Config]
* 		/rosout [rosgraph_msgs/Log]
*	Services: <BR>
* 		/img_echo/compressed/set_parameters
* 		/img_echo/compressedDepth/set_parameters
* 		/img_echo/get_loggers
* 		/img_echo/set_logger_level
* 		/img_echo/theora/set_parameters
*
*	Client Services: <BR>
		None
*
*	Action Services: <BR>
*    	None
*
*	Description: <BR>
*		Briefly, this node reads the input image from the robot's camera.
*		Secondly, it  print it on a floating window, namely DetectiCAm, by means
*		of a cv_ptr; Thirdly it publish the video stream!
*
*
*
*/
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include "sensor_msgs/Image.h"

#include <string>
#include <vector>

/// window name
static const std::string OPENCV_WINDOW = "DetectiCam";

class img_echo_class
{
public:

	/** img_echo class constructor */
	img_echo_class() : it(nh)
	{
		// open the channels
		image_sub = it.subscribe(in_topic , 1, &img_echo_class::image_callback, this);
		image_pub = it.advertise(out_topic, 1);

		// open the window
		cv::namedWindow(OPENCV_WINDOW);
	}

	/** img_echo node class destructor (just close the window) */
	~img_echo_class()
	{
		// destroy the window
		cv::destroyWindow(OPENCV_WINDOW);
	}

	/** receive the image, print it on a window, then echo the image */
	void image_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		// read the input image
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		// write it on the window
		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(30);

		// publish the video stream ("echo")
		image_pub.publish(cv_ptr->toImageMsg());
	}

private:

	/// class node handle
	ros::NodeHandle nh;

	/// image input topic
	const std::string in_topic = "/robot/camera1/image_raw";

	/// image output topic
	const std::string out_topic = "/img_echo";

	/// the image transport handle
	image_transport::ImageTransport it;

	/// image transport subscriber from Gazebo
	image_transport::Subscriber image_sub;

	/// image transport publisher to the architecture
	image_transport::Publisher image_pub;
};


/** main function */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "img_echo");

	img_echo_class echo;
	ros::spin();

	return 0;
}
