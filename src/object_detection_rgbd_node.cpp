#include <iostream>
#include <chrono>
#include <ros/console.h>
#include "image_converter.hpp"

using namespace std;
using namespace ros;
using namespace cv;

int depth_sampling;
Scalar color_Lab;
bool debug_lab,debug_bin,debug_gray,debug_thresh,debug_edg,debug_dst,debug_cmap;
int blur_size,bin_thresh,edg_thresh1,edg_thresh2,cost_max,fov_y,fov_x,z0;
float blur_sigma,cost_thresh,r0;
string time_delay_csv_path;
int main(int argc, char **argv){
	string template_path;//need to be passed by constructor
	init(argc, argv, "object_detection_rgbd");
	NodeHandle nh;
	string cam_topic,location_topic;
	int target_color_L,target_color_a,target_color_b;
	if (!(nh.getParam("object_detection_rgbd/camera_topic", cam_topic) 
		&& nh.getParam("object_detection_rgbd/location_topic", location_topic) 
		&& nh.getParam("object_detection_rgbd/time_delay_csv_path", time_delay_csv_path) 
		&& nh.getParam("object_detection_rgbd/template_path", template_path) 
		&& nh.getParam("object_detection_rgbd/depth_sampling", depth_sampling) 
		&& nh.getParam("object_detection_rgbd/target_color_L", target_color_L)
		&& nh.getParam("object_detection_rgbd/target_color_a", target_color_a) 
		&& nh.getParam("object_detection_rgbd/target_color_b", target_color_b)
				
		&& nh.getParam("object_detection_rgbd/DEBUG_LAB", debug_lab) 
		&& nh.getParam("object_detection_rgbd/DEBUG_BIN", debug_bin) 
		&& nh.getParam("object_detection_rgbd/DEBUG_GRAY", debug_gray) 
		&& nh.getParam("object_detection_rgbd/DEBUG_THRESH", debug_thresh) 
		&& nh.getParam("object_detection_rgbd/DEBUG_EDG", debug_edg) 
		&& nh.getParam("object_detection_rgbd/DEBUG_DST", debug_dst) 
		&& nh.getParam("object_detection_rgbd/DEBUG_CMAP", debug_cmap) 
		
		&& nh.getParam("object_detection_rgbd/BLUR_SIZE", blur_size) //int
		&& nh.getParam("object_detection_rgbd/BLUR_SIGMA", blur_sigma) //float
		&& nh.getParam("object_detection_rgbd/BIN_THRESH", bin_thresh) //int
		&& nh.getParam("object_detection_rgbd/EGD_THERSH1", edg_thresh1) //int
		&& nh.getParam("object_detection_rgbd/EGD_THERSH2", edg_thresh2) //int
		&& nh.getParam("object_detection_rgbd/COST_THRESH", cost_thresh) //float
		
		&& nh.getParam("object_detection_rgbd/COST_MAX", cost_max)
		
		&& nh.getParam("object_detection_rgbd/FOV_X", fov_x)
		&& nh.getParam("object_detection_rgbd/FOV_Y", fov_y)
		&& nh.getParam("object_detection_rgbd/Z0", z0)
		&& nh.getParam("object_detection_rgbd/R0", r0)
		
		)){
		ROS_FATAL("Problem reading config from yaml file... exiting !");
		exit(1);
	}

	
	color_Lab = Scalar(target_color_L,target_color_a,target_color_b);
	ImageConverter ic(cam_topic,template_path,location_topic);
	
	spin();
	return 0;
}
