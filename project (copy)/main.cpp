#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream> 
#include <unistd.h>
#include <opencv2/cudawarping.hpp>
#include "opencv2/cudaarithm.hpp"

#include "cpu_src.hpp"
#include "gpu_src.hpp"
#include "opengl_src.hpp"
#include "main_inc.hpp"

int main(){
	init_opengl();
	init_gpu_6d(1.0, 10);
	
	rs2::pipeline p;
	rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH , 1280, 720, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	p.start(cfg);
	std::ofstream data_file("data.csv");
	int n=0;
	while(true){
		rs2::frameset frames = p.wait_for_frames();          
		rs2::frame fdepth = frames.get_depth_frame();
		rs2::frame fcolor = frames.get_color_frame();
		const int wd = fdepth.as<rs2::video_frame>().get_width();
		const int wc = fcolor.as<rs2::video_frame>().get_width();
		const int hd = fdepth.as<rs2::video_frame>().get_height();
		const int hc = fcolor.as<rs2::video_frame>().get_height();
		cv::Mat depth(cv::Size(wd, hd), CV_16U, (void*)fdepth.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat color(cv::Size(wc, hc), CV_8UC3, (void*)fcolor.get_data(), cv::Mat::AUTO_STEP);
		
		auto start = std::chrono::high_resolution_clock::now();
		
		cv::Mat color1, color2;
		cv::pyrDown(color,color1);
		cv::pyrDown(color1,color2);
		
		Mat edge = get_edge(color2);
		bitwise_not(edge, edge);
		
		cv:Mat edge_distance, lbl;
		cv::distanceTransform(edge, edge_distance, lbl, cv::DIST_L2, 3);
		edge_distance.convertTo(edge_distance, CV_8U);
		
		//locate_cuda_6d(edge_distance);
		
		
		std::chrono::duration<double>  d = std::chrono::high_resolution_clock::now()-start;
		
		
		//show_text(&edge_distance, 1/d.count(), float scale, Size template_size, KeyPoint k);
		//cv::imshow("depth", _depth);
		//cv::imshow("color", _color);
		cv::imshow("edge_distance", edge_distance);
		//cv::imshow("lbl", lbl*10);
		cv::waitKey(27);
		//if(n<100)n++;else break;
	}
	data_file.close();
	return 0;
}
