#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <iostream>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <opencv2/cudawarping.hpp>
#include "opencv2/cudaarithm.hpp"

int main(){

	rs2::pipeline p;
	rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH , 1280, 720, RS2_FORMAT_Z16, 15);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
	p.start(cfg);

	
	while(true){
		rs2::frameset frames = p.wait_for_frames();          
		rs2::frame depth = frames.get_depth_frame();
		rs2::frame color = frames.get_color_frame();
		const int wd = depth.as<rs2::video_frame>().get_width();
		const int wc = color.as<rs2::video_frame>().get_width();
		const int hd = depth.as<rs2::video_frame>().get_height();
		const int hc = color.as<rs2::video_frame>().get_height();
		cv::Mat _depth(cv::Size(wd, hd), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
		cv::Mat _color(cv::Size(wc, hc), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
		
		auto start = std::chrono::high_resolution_clock::now();
		cv::cuda::Stream s0, s1;
		cv::cuda::GpuMat g_depth, g_color, g_color1, g_color2;
		/*
		g_depth.upload(_depth,s0);
		cv::cuda::threshold(g_depth, g_depth, 2000, 0, cv::THRESH_TOZERO_INV,s0);
		cv::cuda::normalize(g_depth,g_depth,0,255,cv::NORM_MINMAX, CV_8U,cv::noArray(),s0);
		g_depth.convertTo(g_depth,CV_8U,s0);
		g_depth.download(_depth,s0);
		*/
		g_color.upload(_color,s1);
		cv::cuda::pyrDown(g_color,g_color1,s1);
		cv::cuda::pyrDown(g_color1,g_color2,s1);
		g_color2.download(_color,s1);
		
		s0.waitForCompletion();
		s1.waitForCompletion();
		
		std::chrono::duration<double>  d = std::chrono::high_resolution_clock::now()-start;
		std::cout<<d.count()<<std::endl;
		
		
		cv::imshow("depth", _depth);
		cv::imshow("color", _color);
		cv::waitKey(27);
	}
	return 0;
}
