#include "cpu_src.hpp"
#include "gpu_src.hpp"
#include "opengl_src.hpp"
#include "main_inc.hpp"

using namespace std;


//Variable to control the execution of the OPENGL thread

int main(int argc, char** argv){
	Mat edge_distance;
	frame color_frame;
	/*------------------------Initialization--------------------------*/
	init_opengl();
	Size template_size = init_gpu("./train1.jpg",1,10);//make an other function for the 6D estimation
	
	init_cpu("train1.jpg",1,15);
	pipeline pipe = init_cam();
	while(true){
		auto start = chrono::high_resolution_clock::now();
		/*----------------------Processing----------------------------*/
		color_frame = pipe.wait_for_frames().get_color_frame();	//--0---
		Mat camera_frame(Size(1280, 720), CV_8UC3, (void*)
				color_frame.get_data(), Mat::AUTO_STEP);		//---0---
					
		edge_distance = get_edge_distance(camera_frame);		//---1---
		float scale = locate_cuda(&edge_distance);			//---2---
		KeyPoint keypoint = get_center(edge_distance);			//---3---
		/*--------------------------Output----------------------------*/
		chrono::duration<double>  d = chrono::high_resolution_clock::now()-start;
		show_text(&camera_frame,d.count(),scale,template_size,keypoint);//>:(
		imshow("Webcam", camera_frame);
		if(waitKey(1) == 27 ) break;
	}
	return 0;
}
