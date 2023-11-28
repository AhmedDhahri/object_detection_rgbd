#ifndef CPU_SRC_HPP
	#define CPU_SRC_HPP
	
	#include <opencv2/opencv.hpp>
	#include <librealsense2/rs.hpp>
	
	using namespace rs2;
	using namespace std;
	using namespace cv;

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * function: Initialise realsense camera parameters.
	 * return: Pipeline object to capture frames from camera.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	pipeline init_cam();

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * function: Initialise CPU calculations parameters.
	 * return: Pipeline object to capture frames from camera.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void init_cpu(string template_path, float _z0, int _depth_sampling);	
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: pointer to an RGB image
	 * function: Extract edges from source image using canny edge detect-
	 * 			or and color segmentation.
	 * return: Gray image caintaining edges.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Mat get_edge(Mat *src);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			RGB image.
	 * function: extarct edges using the get_edge function then extarct 
	 * 			edge distance.
	 * reurn: Gray image of the same size.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Mat get_edge_distance(Mat src);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: Template matching output
	 * function: Get the centers of the blobs generated 
	 * 			by the template matching.
	 * return: keypoint object contains the image location in X,Y and 
	 * 			the cost
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	KeyPoint get_center(Mat src);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: pointer to the camera frame.
	 * 			frame_rate: the processing time of the frame.
	 * 			scale: the scaling factor of the temlate.
	 * 			template_size: size of the matced template.
	 * 			k: the detected key point.
	 * function: Calculate the location and print it with the rest of 
	 * 			the information downside the camera stream window
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void show_text(Mat *src, float frame_rate, float scale, Size template_size, KeyPoint k);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: A pointer to the Mat image to be processed
	 * function: Do the template matching on the image and each 
	 * 				upsampled template in the pyramid aray and 
	 * 				then extract the location with the lieast 
	 * 				response. Then, download it in the src pointer.
	 * return: the scale of the mached template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	float locate_cpu(Mat* src);
	
		
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			template_path: Path of template file.
	 * function: create a bunch of upsampled image from the template
	 * 				image and store them in the pyramid array.
	 * return: the size of the original template.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	static Size templates_cpu(string template_path);
	
	Mat get_dft_filter(Mat I, Mat T);
	Mat match(Mat src, Mat pat);
#endif
