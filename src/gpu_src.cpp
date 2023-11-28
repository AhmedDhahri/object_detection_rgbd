#include "cpu_src.hpp"
#include "gpu_src.hpp"
#define DEBUG

extern int depth_sampling;
extern Scalar color_Lab;

Size template_size;
cuda::GpuMat* pyramid;

cuda::GpuMat get_edge_cuda(cuda::GpuMat src, cuda::Stream s){
	Ptr<cuda::CannyEdgeDetector> edg_detect = cuda::createCannyEdgeDetector(5,68, 3, false);   
	Ptr<cuda::Filter> filter_gauss  = cuda::createGaussianFilter(CV_8UC3 ,CV_32FC3 , Size(5,5),5,5,BORDER_DEFAULT,-1);
	src.convertTo(src,CV_8UC3,s);
	filter_gauss->apply(src,src,s);
	cvtColor(src,src, COLOR_BGR2Lab,0,s);
	cuda::subtract(src,Scalar(14,12,18),src,noArray(),-1,s);//prob
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

void templates_pyramid(string template_path){
	pyramid = new cuda::GpuMat[depth_sampling];
	cuda::Stream stream1[depth_sampling];
	Mat pat = imread(template_path.c_str());
	template_size = pat.size();
	cuda::GpuMat d_templ;
	d_templ.upload(pat);
	for(int i=0;i<depth_sampling;i++){
		cuda::resize(d_templ,pyramid[i],Size(0,0),1.5+((double)i/depth_sampling),1.5+((double)i/depth_sampling),INTER_CUBIC,stream1[i]);
		pyramid[i] = get_edge_cuda(pyramid[i],stream1[i]);
	}
	for(int i=0;i<depth_sampling;i++)
		stream1[i].waitForCompletion();
}

int locate_cuda(Mat* src){
	Ptr<cuda::TemplateMatching> tmpl_match = cuda::createTemplateMatching(CV_8U, TM_CCORR_NORMED);
	int sel;double minVal,maxVal;Point minLoc,maxLoc;
	double sel_min[depth_sampling];
	Mat sel_min_cpu[depth_sampling];
	cuda::Stream stream[depth_sampling];
	cuda::GpuMat gpu_cand[depth_sampling];
	cuda::GpuMat gpu_src;
	cuda::GpuMat sel_min_gpu;
	gpu_src.upload(*src);
	for(int i=0;i<depth_sampling;i++){
		tmpl_match->match(gpu_src, pyramid[i], gpu_cand[i],stream[i]);
		//cuda::threshold(gpu_cand[i],gpu_cand[i],0.09,1.0,THRESH_TRUNC,stream[i]);//when converting Mat 
		//from float to byte values above 255 will be thersholded.
	}
	for(int i=0;i<depth_sampling;i++){
		stream[i].waitForCompletion();
		cuda::minMaxLoc(gpu_cand[i],&minVal,&maxVal,&minLoc,&maxLoc,noArray());
		sel_min[i] = minVal;
	}
	
	sel = (min_element(sel_min,sel_min+depth_sampling)-sel_min);
	gpu_cand[sel].download(*src);
	return sel;
}

