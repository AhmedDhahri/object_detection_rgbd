#include "image_converter.hpp"

extern ofstream csvfile;
extern int cost_max;
ImageConverter::ImageConverter(const string cam_topic, string template_path, const string loc_topic):it_(nh_){
	init();
	templates_pyramid(template_path);
	image_sub_ = it_.subscribe(cam_topic, 1,&ImageConverter::image_capture, this);
	pub = nh_.advertise<object_detection_rgbd::location>(loc_topic, 30);
}
ImageConverter::~ImageConverter(){
	csvfile.close();
}

void ImageConverter::image_capture(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	buffer.cam = cv_ptr->image;
	buffer.process();
}

void Buffer::process(){
	start_time = chrono::high_resolution_clock::now();
	src = get_edge_distance(cam);
	sel = locate_cuda(&src);
	k = get_center(src);
	time = 1000*((chrono::duration<double>)(chrono::high_resolution_clock::now()-start_time)).count();
	loc = cam_output(cam,time,sel,k);
	publish_location(k.response, loc);
}


void publish_location(uint8_t cost, _3dloc loc){
	object_detection_rgbd::location _loc;
	_loc.roll=0;_loc.pitch=0;_loc.yaw=0;
	if (cost < cost_max){
		_loc.x = loc.x;_loc.y = loc.y;_loc.z = loc.z;
	}
	else{
		_loc.x = 0;_loc.y = 0;_loc.z = 300;
	}
	pub.publish(_loc);
}


