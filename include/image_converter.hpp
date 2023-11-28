#ifndef IMAGE_CONVERTER
	#define IMAGE_CONVERTER
	#include <ros/ros.h>
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>
	#include <sensor_msgs/image_encodings.h>
	#include <object_detection_rgbd/location.h>
	#include <ros/console.h>
	
	#include <unistd.h>
	#include <thread>
	#include <mutex>

	#include "cpu_src.hpp"
	#include "gpu_src.hpp"
	
	
	using namespace cv;
	using namespace std;
	using namespace ros;
		
	Publisher pub;
	void publish_location(uint8_t cost, _3dloc loc);
	
	class Buffer{
		public:
		void process();
		
		chrono::high_resolution_clock::time_point start_time;
		int time;
		KeyPoint k;
		_3dloc loc;
		Mat src;
		Mat cam;
		int sel;
	};
	
	Buffer buffer;
	class ImageConverter{
		
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;

	public:
		ImageConverter(const string cam_topic, string template_path, const string loc_topic);
		~ImageConverter();
		void image_capture(const sensor_msgs::ImageConstPtr& msg);
		
	};
	

#endif


/*data{Mat,sel}
buffer{data,mutex1,mutex2,mutex3}

Program_section1{
	//select and lock mutex1
	//proc
	//set time
	//unlocj mitex2
}
Program_section2{
	//select and lock mutex2
	//proc
	//unlocj mitex3
}
Program_section3{
	//select and lock mutex3
	//proc
	//unlocj mitex1
}
*/
