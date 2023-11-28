#include "gpu_src.hpp"
#include "opengl_src.hpp"

static int depth_sampling;
static float z0;//easy camera calibration
static cuda::GpuMat *pyramid;
static template_gpu *pyramid_6d;
extern vector<int> yaw_step;//{0,45,90};
extern vector<int> pitch_step;//{-45,0,45};

extern vector<template_i> templates;
extern mutex idle_func_blocking;

Size init_gpu(string template_path, float _z0, int _depth_sampling){
	//print cuda device info
	cuda::printShortCudaDeviceInfo(cuda::getDevice());
	//declare pointers to algorithms for cuda computations
	Ptr<cuda::CannyEdgeDetector> edg_detect = cuda::createCannyEdgeDetector(5,68, 3, false);   
	depth_sampling = _depth_sampling;z0 = _z0;
	return templates_pyramid(template_path);
}

Size init_gpu_6d(float _z0, int _depth_sampling){
	//print cuda device info
	cuda::printShortCudaDeviceInfo(cuda::getDevice());
	depth_sampling = _depth_sampling;z0 = _z0;
	template_gpu* templates_gpu = new template_gpu[templates.size()];
	cuda::Stream* s = new cuda::Stream[templates.size()];
	for(int i=0;i<templates.size();i++){
		templates_gpu[i].mat.upload(templates[i].temp,s[i]);
		templates_gpu[i].yaw = yaw_step[i/yaw_step.size()];
		templates_gpu[i].pitch = pitch_step[i%yaw_step.size()];
	}
	for(int i=0;i<templates.size();i++)
		s[i].waitForCompletion();
	return templates_pyramid_6d(templates_gpu);
}
	
cuda::GpuMat get_edge_cuda(cuda::GpuMat src, cuda::Stream s){//extract edge directly before resizing
	//declare pointers to algorithms for cuda computations
	Ptr<cuda::CannyEdgeDetector> edg_detect = cuda::createCannyEdgeDetector(5,68, 3, false);   
	Ptr<cuda::Filter> filter_gauss  = cuda::createGaussianFilter(CV_8UC3 ,CV_32FC3 , Size(5,5),5,5,BORDER_DEFAULT,-1);
	
	src.convertTo(src,CV_8UC3,s);
	//Eliminate small edglets from noise
	filter_gauss->apply(src,src,s);
	cvtColor(src,src, COLOR_BGR2Lab,0,s);
	cuda::subtract(src,Scalar(TARGET_COLOR_WC),src,noArray(),-1,s);
	cuda::pow(src,2,src,s);;
	cuda::GpuMat tmp[3];
	cuda::split(src,tmp,s);
	cuda::add (tmp[0],tmp[2],src,noArray(),-1,s);
	cuda::add (tmp[2],src,src,noArray(),-1,s);
	cuda::sqrt(src,src,s);
	cuda::normalize(src,src,0,255,NORM_MINMAX,CV_8U,noArray(),s);
	edg_detect->detect(src,src,s);
	return src;
}

static Size templates_pyramid(string template_path){
	pyramid = new cuda::GpuMat[depth_sampling];
	//streams to organize parallel computaions.
	cuda::Stream stream1[depth_sampling];
	cuda::GpuMat d_templ;
	Mat pat = imread(template_path.c_str());
	Size template_size = pat.size();
	d_templ.upload(pat);
	for(int i=0;i<depth_sampling;i++){
		cuda::resize(d_templ,pyramid[i],Size(0,0),z0+((double)i/depth_sampling),z0+((double)i/depth_sampling),INTER_LINEAR,stream1[i]);
		pyramid[i] = get_edge_cuda(pyramid[i],stream1[i]);
	}
	//wait for GPU computations to finish before going forward.
	for(int i=0;i<depth_sampling;i++)
		stream1[i].waitForCompletion();
	return template_size;
}

static Size templates_pyramid_6d(template_gpu* templates_gpu){
	Ptr<cuda::CannyEdgeDetector> edg_detect = cuda::createCannyEdgeDetector(5,68, 3, false); 
	pyramid_6d = new template_gpu[depth_sampling*templates.size()];
	cuda::Stream s[depth_sampling*templates.size()];
	for(int i=0;i<templates.size();i++)
		for(int j=0;j<depth_sampling;j++){
			/*cout<<templates_gpu[i*depth_sampling+j].mat.channels()<<" "
			<<templates_gpu[i*depth_sampling+j].mat.depth()<<endl;*/
			cuda::resize(
						templates_gpu[i].mat,
						pyramid_6d[i*depth_sampling+j].mat,
						Size(0,0),z0+((double)j/depth_sampling),
						z0+((double)j/depth_sampling),
						INTER_LINEAR,
						s[i*depth_sampling+j]);
			edg_detect->detect(pyramid_6d[i*depth_sampling+j].mat,pyramid_6d[i*depth_sampling+j].mat,s[i*depth_sampling+j]);
			pyramid_6d[i*depth_sampling+j].pitch = templates_gpu[i].pitch;
			pyramid_6d[i*depth_sampling+j].yaw = templates_gpu[i].yaw;
			pyramid_6d[i*depth_sampling+j].scale = z0+((double)j/depth_sampling);
		}
	for(int i=0;i<depth_sampling*templates.size();i++)
		s[i].waitForCompletion();
	return templates_gpu[0].mat.size();
}

float locate_cuda(Mat *src){
	//declare pointers to algorithms for cuda computations
	Ptr<cuda::TemplateMatching> tmpl_match = cuda::createTemplateMatching(CV_8U, TM_CCORR_NORMED);
	double minVal,maxVal;Point minLoc,maxLoc;
	//streams to organize parallel computaions.
	cuda::Stream stream[depth_sampling];
	//Costmap candadates (store all costmaps)
	cuda::GpuMat gpu_cand[depth_sampling];
	cuda::GpuMat sel_min_gpu, gpu_src;
	Mat sel_min_cpu[depth_sampling];
	double sel_min[depth_sampling];
	int selected_frame = 0;
	//copy Mat to graphic memory
	gpu_src.upload(*src);
	for(int i=0;i<depth_sampling;i++){
		tmpl_match->match(gpu_src, pyramid[i], gpu_cand[i],stream[i]);
	}
	
	//wait for GPU computations to finish before going forward.
	//select min value form every frame
	for(int i=0;i<depth_sampling;i++){
		stream[i].waitForCompletion();
		cuda::minMaxLoc(gpu_cand[i],&minVal,&maxVal,&minLoc,&maxLoc,noArray());
		sel_min[i] = minVal;
	}
	//get frame index of the minimum from min values
	selected_frame = (min_element(sel_min,sel_min+depth_sampling)-sel_min);
	//calculate the scaling factor
	float scale = z0+((double)selected_frame/depth_sampling);
	//copy image from graphic memory
	gpu_cand[selected_frame].download(*src);
	return scale;
}

float* locate_cuda_6d(Mat *src){
	//declare pointers to algorithms for cuda computations
	Ptr<cuda::TemplateMatching> tmpl_match = cuda::createTemplateMatching(CV_8U, TM_CCORR_NORMED);
	double minVal,maxVal;Point minLoc,maxLoc;
	//streams to organize parallel computaions.
	cuda::Stream s[depth_sampling*templates.size()];
	//Costmap candadates (store all costmaps)
	cuda::GpuMat gpu_cand[depth_sampling*templates.size()];
	cuda::GpuMat sel_min_gpu, gpu_src;
	Mat sel_min_cpu[depth_sampling*templates.size()];
	double sel_min[depth_sampling*templates.size()];
	int selected_frame = 0;
	//copy Mat to graphic memory
	gpu_src.upload(*src);
	for(int i=0;i<depth_sampling*templates.size();i++){
		tmpl_match->match(gpu_src, pyramid_6d[i].mat, gpu_cand[i],s[i]);
	}
	
	//wait for GPU computations to finish before going forward.
	//select min value form every frame
	for(int i=0;i<depth_sampling*templates.size();i++){
		s[i].waitForCompletion();
		cuda::minMaxLoc(gpu_cand[i],&minVal,&maxVal,&minLoc,&maxLoc,noArray());
		sel_min[i] = minVal;
	}
	//get frame index of the minimum from min values
	selected_frame = (min_element(sel_min,sel_min+depth_sampling*templates.size())-sel_min);
	//Get scale and angle
	float* loc_6d = new float[3];
	loc_6d[0] = pyramid_6d[selected_frame].yaw;
	loc_6d[1] = pyramid_6d[selected_frame].pitch;
	loc_6d[2] = pyramid_6d[selected_frame].scale;
	cout<<pyramid_6d[selected_frame].yaw<<" "<<pyramid_6d[selected_frame].pitch<<" "<<pyramid_6d[selected_frame].scale<<endl;
	//copy image from graphic memory
	gpu_cand[selected_frame].download(*src);
	return loc_6d;
}

