#ifndef GPU_SRC_HPP
	#define GPU_SRC_HPP
	
	#include <opencv2/opencv.hpp>
	#include "opencv2/core/cuda.hpp"
	#include "opencv2/cudaimgproc.hpp"
	#include "opencv2/cudaarithm.hpp"
	#include "opencv2/cudawarping.hpp"
	#include "opencv2/cudafilters.hpp"
	#include <unistd.h>
	
	using namespace std;
	using namespace cv;
	
	
		class template_gpu{
		public:
		
		cuda::GpuMat mat;
		float scale;
		float yaw;
		float pitch;
		float roll;
	};
	

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			template_path: Path of the template_file (.obj or .jpg)
	 * 			_z0: initial depth of the object ***
	 * function: Initialize parameters of the GPU calculation.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Size init_gpu(string template_path, float _z0, int _depth_sampling);
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			template_path: Path of the template_file (.obj or .jpg)
	 * 			_z0: initial depth of the object ***
	 * function: Initialize parameters of the GPU calculation.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Size init_gpu_6d(float _z0, int _depth_sampling);
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: RGB image
	 * 			s: Cuda stream in the pipeline
	 * function: Extract edges from source image linking the operation 
	 * 				to a cuda stream.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	cuda::GpuMat  get_edge_cuda(cuda::GpuMat src, cuda::Stream s);	
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			template_path: Path of template file.
	 * function: create a bunch of upsampled image from the template
	 * 				image and store them in the pyramid array.
	 * return: the size of the original template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	static Size templates_pyramid(string template_path);
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			template_path: Path of template file.
	 * function: create a bunch of upsampled image from the template
	 * 				image and store them in the pyramid array.
	 * return: the size of the original template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	static Size templates_pyramid_6d(template_gpu* templates_gpu);
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: A pointer to the Mat image to be processed
	 * function: Do the template matching on the image and each 
	 * 				upsampled template in the pyramid aray and 
	 * 				then extract the location with the lieast 
	 * 				response. Then, download it in the src pointer.
	 * return: the scale of the mached template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	float locate_cuda(Mat* src);
	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: A pointer to the Mat image to be processed
	 * function: Do the template matching on the image and each 
	 * 				upsampled template in the pyramid aray and 
	 * 				then extract the location with the lieast 
	 * 				response. Then, download it in the src pointer.
	 * return: the scale of the mached template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	float* locate_cuda_6d(Mat* src);
#endif
