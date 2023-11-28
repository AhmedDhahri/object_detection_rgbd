#ifndef CPU_SRC_HPP
	#define CPU_SRC_HPP
	#include <opencv2/core/core_c.h>
	#include <opencv2/opencv.hpp>
	#include <opencv2/features2d.hpp>
	
	#include <fstream>
	
	using namespace std;
	using namespace cv;
	
	
	typedef struct{
		int x;
		int y;
		int z;
	}_3dloc;
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * function: Initialise the blob detection algorithme.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void init();
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: RGB image
	 * function: Extract edges from source image.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Mat get_edge(Mat src);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: Edge extracted from RGB image 
	 * function: Do the chamfer matching cost map. Assigne 
	 * 				to every picel its distance to the 
	 * 				nearest edge line.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	Mat get_edge_distance(Mat src);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * function:
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	_3dloc cam_output(Mat src, int frame_rate, int sel, KeyPoint k);
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * 			src: Template matching output
	 * function: Get the centers of the blobs generated 
	 * 			by the template matching.
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	KeyPoint get_center(Mat src);
#endif
