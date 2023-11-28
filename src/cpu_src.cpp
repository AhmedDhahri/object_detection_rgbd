#include "cpu_src.hpp"
#define DEBUG
extern int depth_sampling;
extern Scalar color_Lab;
extern Size template_size;

extern bool debug_lab,debug_bin,debug_gray,debug_thresh,debug_edg,debug_dst,debug_cmap;
extern int blur_size,bin_thresh,edg_thresh1,edg_thresh2,cost_max,fov_y,fov_x,z0;
extern float blur_sigma,cost_thresh,r0;
extern string time_delay_csv_path;
ofstream csvfile;

Ptr<SimpleBlobDetector> sbd;
Ptr<SimpleBlobDetector> sbd_params(){
	SimpleBlobDetector::Params params = SimpleBlobDetector::Params();
	params.filterByColor = true;
	params.blobColor = 0;
	
    params.minThreshold = 0;
    params.maxThreshold = 230;
	params.filterByArea = true;
	params.minArea = 150;
	params.maxArea = 2500;
	params.filterByCircularity = true;
	params.minCircularity = 0.5;
	params.maxCircularity = 1.0;
	params.filterByConvexity = false;
	params.filterByInertia = false;
	Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
	return detector;
}

void init(){
	csvfile.open(time_delay_csv_path.c_str(),ios::out);
	sbd = sbd_params();
}

Mat get_edge(Mat res){//Convert to 32F before processing because Lab conversion calculation
	Mat src, ini,src1;
	cvtColor(res,src,COLOR_BGR2Lab);
#ifdef DEBUG
	if(debug_lab)imshow("CIE-LAB",src);
#endif
	subtract(src, color_Lab, src);//prob
	src.convertTo(src, CV_32FC3);
	pow(src, 2.0, src);
	Mat cls[3];split(src, cls);
	src = cls[0] + cls[1] + cls[2];
	sqrt(src,src);
	src.convertTo(src,CV_8U);
#ifdef DEBUG
	if(debug_bin)imshow("Euclidian distance",src);
#endif
	GaussianBlur(src,src, Size(blur_size,blur_size),blur_sigma);
#ifdef DEBUG
	if(debug_gray)imshow("Blur",src);
#endif
	threshold(src,src,bin_thresh,0,THRESH_TRUNC);
#ifdef DEBUG
	if(debug_thresh)imshow("Thersholding",src);
#endif
	Canny(src, src, edg_thresh1,edg_thresh2);
#ifdef DEBUG
	if(debug_edg)imshow("Edge detection",src);
#endif
	return src;
}


Mat get_edge_distance(Mat src){
	register float d1 = 1.5;
	register float d2 = 2.0;
	register float dist0,dist1,dist2,dist3;
	src = get_edge(src);
	bitwise_not(src, src);
	src.convertTo(src, CV_32F);
	
	for(int i=src.rows-2;i>0;i--){
		for(int j=src.cols-2;j>0;j--){
			dist0 = d1 + src.at<float>(i,j+1);
			dist1 = d2 + src.at<float>(i+1,j+1);
			dist2 = d1 + src.at<float>(i+1,j);
			dist3 = d2 + src.at<float>(i-1,j+1);
			src.at<float>(i,j) = min(min(src.at<float>(i,j), min(dist0,dist1)), min(dist2,dist3));
			
		}
	}
	for(int i=1;i<src.rows-1;i++){
		for(int j=1;j<src.cols-1;j++){
			dist0 = d1 + src.at<float>(i,j-1);
			dist1 = d2 + src.at<float>(i-1,j-1);
			dist2 = d1 + src.at<float>(i-1,j);
			dist3 = d2 + src.at<float>(i+1,j-1);
			src.at<float>(i,j) = min(min(src.at<float>(i,j), min(dist0,dist1)), min(dist2,dist3));
		}
	}
	src.convertTo(src, CV_8U);
#ifdef DEBUG
	if(debug_dst)imshow("Edge distance",src);
#endif
	return src;
}

_3dloc cam_output(Mat src, int frame_rate, int sel, KeyPoint k){
	Size s;
	Mat padded;_3dloc loc;
	if(k.response < cost_max){
		s.width = template_size.width*(1.5+sel*(1.0/depth_sampling));//cols
		s.height = template_size.height*(1.5+sel*(1.0/depth_sampling));//rows
		Rect r(k.pt,s);
		rectangle(src,r,Scalar(255,0,0),1,8,0);
	}
	
	int zr = (z0/(r0+((double)sel/depth_sampling)));//param
	int yr = (k.pt.y - 240 + s.height/2)*((double)zr/fov_y);//param
	int xr = (k.pt.x - 320 + s.width/2)*((double)zr/fov_x);//param
	loc.x=xr;loc.y=yr;loc.z=zr;
	String text =  to_string(frame_rate) + " ms | Loc: x: " 
		+ to_string(xr) 
		+ " cm    y: "
		+ to_string(yr)
		+ " cm    z: "
		+ to_string(zr)
		+ " cm    "
		+ "cost: "
		+ to_string((int)k.response);
    int m = src.rows + 30;
    copyMakeBorder(src, src, 0, m - src.rows, 0, 0, BORDER_CONSTANT, Scalar::all(0));
	putText(src, text, Point(5,m-5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1, 12, false );
	imshow("Camera", src);
	waitKey(3);
	csvfile<<frame_rate<<"\n";
	return loc;
}

KeyPoint get_center(Mat src){
	Mat res = src * (int)(255/cost_thresh);
	res.convertTo(res,CV_8U);
#ifdef DEBUG
	if(debug_cmap)imshow("Costmap",src);
#endif
	vector<KeyPoint> keypoints;
	sbd->detect(res,keypoints);
	if(keypoints.size()>0){
		keypoints[0].response = res.at<uint8_t>(keypoints[0].pt.y,keypoints[0].pt.x);
		return keypoints[0];
	}
	KeyPoint k;
	k.pt.x = 0;
	k.pt.y = 0;
	k.response = 255;
	return k;
}

