#include "cpu_src.hpp"

static int depth_sampling = 10;
static float z0 = 1.0;
static Mat* pyramid;
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

pipeline init_cam(){
	pipeline pipe;
	config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH , 1280, 720, RS2_FORMAT_Z16, 15);
	pipe.start(cfg);
	return pipe;
}

void init_cpu(string template_path, float _z0, int _depth_sampling){
	cout<<"OpenCV version: "<<CV_VERSION<<endl<<endl;
	depth_sampling = _depth_sampling;z0 = _z0;
	pyramid = new Mat[depth_sampling];
	sbd = sbd_params();
	//return init_cam();
}

Mat get_edge(Mat res){//Convert to 32F before processing because Lab conversion calculation
	Mat src;
	cvtColor(res,src,COLOR_BGR2Lab);
	if(DEBUG_LAB)imshow("Lab",src);//
	src.convertTo(src, CV_32FC3);
	subtract(src, Scalar(TARGET_COLOR), src);
	pow(src, 2.0, src);
	Mat cls[3];split(src, cls);
	src = cls[0] + cls[1] + cls[2];
	sqrt(src,src);
	src.convertTo(src,CV_8U);
	/*if(DEBUG_BIN)imshow("Binary",src);
	GaussianBlur(src,src, Size(BLUR_SIZE), BLUR_SIGMA);
	if(DEBUG_GRAY)imshow("Gray",src);
	threshold(src,src,BIN_THRESH,0,THRESH_TRUNC);
	if(DEBUG_THRESH)imshow("Threshold",src);*/
	Canny(src, src, EGD_THERSH);
	if(DEBUG_EDG)imshow("Edges",src);
	return src;
}

Mat get_edge_distance(Mat src){
	register float d1 = 1.5;//Parametre
	register float d2 = 2.0;//Parametre
	register float dist0,dist1,dist2,dist3;
	src = get_edge(src);
	bitwise_not(src, src);//remove this and search for max not min
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
	//To increase distance when the frame is occluded
	normalize(src,src,0,255,NORM_MINMAX, CV_8U);
	if(DEBUG_DST)imshow("Edge distance",src);
	return src;
}

KeyPoint get_center(Mat src){
	Mat res = src * (float)((float)255.0/COST_THRESH);
	res.convertTo(res,CV_8U);
	if(DEBUG_CMAP)imshow("costmap",res);
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

void show_text(Mat *src, float frame_rate, float scale, Size template_size, KeyPoint k){
	Mat padded;
	String text = "";
	int zr = 75/scale;
	int yr = (k.pt.y - 240 + template_size.height/2)*((double)zr/500);//separate function -- parameter
	int xr = (k.pt.x - 320 + template_size.width/2)*((double)zr/500);//separate function -- parameter
	if ((zr>1000)||(zr<-1000))zr=0;
	if ((yr>1000)||(yr<-1000))yr=0;
	if ((xr>1000)||(xr<-1000))xr=0;
	if(frame_rate !=0)
		text =  to_string((int)(1000*frame_rate)) + " ms | Loc: x: " 
			+ to_string(xr) 
			+ " cm    y: "
			+ to_string(yr)
			+ " cm    z: "
			+ to_string(zr)
			+ " cm    "
			+ "cost: "
			+ to_string((int)k.response);
	if(k.response < 110)
		rectangle(*src, 
				Point(k.pt.x,k.pt.y), 
				Point(k.pt.x+100*scale,k.pt.y+100*scale),
				Scalar(0,255,0));
	int m = src->rows + 30;
    copyMakeBorder(*src, *src, 0, m - src->rows, 0, 0, BORDER_CONSTANT, Scalar::all(0));
	putText(*src, text, Point(5,m-5), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255), 1, 12, false );
	
}


//------------------------------------------- à regler
static Size templates_pyramid(string template_path){
	pyramid = new Mat[depth_sampling];
	Mat pat = imread(template_path.c_str());
	Size template_size = pat.size();
	for(int i=0;i<depth_sampling;i++){
		resize(pat,pyramid[i],Size(0,0),z0+((double)i/depth_sampling),z0+((double)i/depth_sampling),INTER_LINEAR);
		pyramid[i] = get_edge(pyramid[i]);
	}
	return template_size;
}

float locate_cpu(Mat *src){
	double minVal,maxVal;Point minLoc,maxLoc;
	Mat cand[depth_sampling];
	Mat sel_min_cpu[depth_sampling];
	double sel_min[depth_sampling];
	int selected_frame = 0;
	for(int i=0;i<depth_sampling;i++){
		matchTemplate(*src, pyramid[i], cand[i],TM_CCORR_NORMED);
	}
	
	for(int i=0;i<depth_sampling;i++){
		minMaxLoc(cand[i],&minVal,&maxVal,&minLoc,&maxLoc,noArray());
		sel_min[i] = minVal;
	}
	
	selected_frame = (min_element(sel_min,sel_min+depth_sampling)-sel_min);
	float scale = z0+((double)selected_frame/depth_sampling);
	*src = cand[selected_frame];
	return scale;
}
//------------- Fin

//------------------------------------------- à regler
Mat get_dft_filter(Mat I, Mat T){
	Mat padded, complexI, magI;
		int m = getOptimalDFTSize(I.rows);
		int n = getOptimalDFTSize(I.cols);

		
		copyMakeBorder(T, padded, 0, m - T.rows, 0, n - T.cols, BORDER_CONSTANT, Scalar::all(0));
		Mat planesT[] = {Mat_<float>(padded), Mat::zeros(Size(n,m), CV_32F)};
		merge(planesT, 2, complexI);
		dft(complexI, complexI);
		split(complexI, planesT);
		
		copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));
		Mat planesI[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
		merge(planesI, 2, complexI);
		dft(complexI, complexI);
		split(complexI, planesI);
		
		Mat planesR[] = {Mat_<float>(padded), Mat::zeros(I.size(), CV_32F)};
		multiply(planesI[0],planesT[0],planesR[0]);
		multiply(planesI[1],planesT[1],planesR[1]);
		planesR[0] -= planesR[1];
		multiply(planesI[0],planesT[1],planesR[1]);
		multiply(planesI[1],planesT[0],planesT[0]);
		planesR[1] += planesT[0];
		merge(planesR, 2, complexI);
		idft(complexI, complexI);
		split(complexI, planesR);
		magI = planesR[0];
		//flip(magI,magI,-1);
		return magI;
}

Mat match(Mat src, Mat pat){
	Mat nom = get_dft_filter(src,pat);
	
	Mat src_sq,pat_sq;
	pow(src,2,src_sq);
	pow(pat,2,pat_sq);
	Mat dom_sq = get_dft_filter(src_sq,pat_sq);
	
	Mat dom;
	sqrt(dom_sq,dom);
	
	Mat res;
	divide(nom,dom,res);
	return res;
}
//------------- Fin
