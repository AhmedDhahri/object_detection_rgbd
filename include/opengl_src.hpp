#ifndef GL_SRC_HPP
	#define GL_SRC_HPP
	#include <GL/glew.h>
	#include <GL/glut.h>
	#include <math.h>
	#include <opencv2/opencv.hpp>
	#include <unistd.h>

	#include <vector>
	#include <mutex>
	#include <iostream>

	using namespace std;
	//class containing the vertecies from the CAD file. Will be used to render graphics in opengl
	class vertex_i{
		public:
		vertex_i(float xc,float yc,float zc){
			x=xc;y=yc;z=zc;
		}
		float x; 
		float y;
		float z;
	};
	//class containing fragment normals (every 3(or 4) vertecies form a fragment or a surface plan)
	class normal{
		public:
		normal(float xc,float yc,float zc){
			x=xc;y=yc;z=zc;
		}
		float x; 
		float y;
		float z;
	};
	//class containing the texture of the fragments (not useful in our case)
	class texture{
		public:
		texture(float sc,float tc){
			s=sc;t=tc;
		}
		float s; 
		float t;
	};
	//class containing the in formation of one face of the object: vertex indecies, normals and textures.
	class face{
		public:
		face(){}
		face(int i,int j,int k, int l){
			v1=i; v2=j; v3=k; v4=l;
		}
		void set_normal(int i,int j,int k, int l){
			vn1=i; vn2=j; vn3=k; vn4=l;
		}
		void set_texture(int i,int j,int k, int l){
			vt1=i; vt2=j; vt3=k; vt4=l;
		}
		int v1,v2,v3,v4;
		int vn1,vn2,vn3,vn4;
		int vt1,vt2,vt3,vt4;
	};
	//class containing information about one template: temp->template image (yaw,pitch,roll)-> euler angles of the object
	//roll is not useful for now
	class template_i{
		public:
		template_i(int height, int width, int type, void* image, float r, float p, float y);
		
		template_i(int height, int width, int type, void* image, float p, float y);
		
		template_i(int height, int width, int type, void* image, float y);
		
		template_i(int height, int width, int type, void* image);
		
		template_i(){}
		
		cv::Mat temp;
		float yaw;
		float pitch;
		float roll;
	};
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Parameters: 
	 * function: Initialise OPENGL calculations parameters.
	 * return: 
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void init_opengl();
#endif
	
