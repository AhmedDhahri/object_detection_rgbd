#include "opengl_src.hpp"

vector<vertex_i> v;
vector<normal> vn;
vector<texture> vt;
vector<face> f;
vector<template_i> templates;
vector<tuple<int, int, int>> AOVs = {(70,)};
vector<int> yaw_step{70, 90, 110};//extern
vector<int> pitch_step{170, 180, 190};//extern

mutex idle_func_blocking;//extern**
template_i instant;//extern
double pitch, yaw, roll=180;//extern

bool done = false;
double dist=-2.6;double mouse_x,mouse_y;int mouse_button;
double fovy=42.5, near=0.1, far=100;
unsigned int p=0,q=0;


template_i::template_i(int height, int width, int type, void* image, float r, float p, float y){
	temp = cv::Mat(120,120,CV_8U,image);
	cv::flip(temp,temp,1);
	roll = r;
	pitch = p;
	yaw = y;
}
	
template_i::template_i(int height, int width, int type, void* image, float p, float y){
	temp = cv::Mat(120,120,CV_8U,image);
	cv::flip(temp,temp,1);
	pitch = p;
	yaw = y;
}
		
template_i::template_i(int height, int width, int type, void* image, float y){
	temp = cv::Mat(120,120,CV_8U,image);
	cv::flip(temp,temp,1);
	yaw = y;
}
		
template_i::template_i(int height, int width, int type, void* image){
	temp = cv::Mat(120,120,CV_8U,image);
	cv::flip(temp,temp,1);
}

static void loadobj(string filename){
	char line[100];
	char mark[5];
	char m;
	float x,y,z;
	int x0,y0,z0,x1,y1,z1,x2,y2,z2,x3,y3,z3;
	face tmp;
	
	v.push_back(vertex_i(0,0,0));vn.push_back(normal(0,0,0));vt.push_back(texture(0,0));
	FILE* fp = fopen(filename.c_str(), "r");

	if(!fp){
		cout<< "Couldn't open the CAD file!"<<endl;
		exit(1);
	}

	while(fgets(line, 100, fp)){
		switch(line[0]){
		case 'v':
			if(line[1]==' '){
				sscanf(line, "%s %f %f %f", mark, &x, &y, &z);
				v.push_back(vertex_i(x,y,z));
			}
			else if(line[1]=='n'){
				sscanf(line, "%s %f %f %f", mark, &x, &y, &z);
				vn.push_back(normal(x,y,z));
			}
			else if(line[1]=='t'){
				sscanf(line, "%s %f %f", mark, &x, &y);
				vt.push_back(texture(x,y));
			}
			
			break;
		case 'f':
			sscanf(line,"%s  %d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c%d",
						mark,&x0,&m,&y0,&m,&z0,&m,&x1,&m,&y1,&m,&z1,&m,&x2,&m,&y2,&m,&z2,&m,&x3,&m,&y3,&m,&z3);
			tmp = face(x0,x1,x2,x3);
			tmp.set_normal(y0,y1,y2,y3);
			tmp.set_texture(z0,z1,z2,z3);
			f.push_back(tmp);
			break;
		default:
			break;
		}
	}
}

static void display() {
	glLoadIdentity ();
	glTranslated(0,0,dist);
	glRotated(180,0,0,1);
	glRotated(pitch,1,0,0);
	glRotated(yaw,0,1,0);
	glRotated(roll,0,0,1);
    gluLookAt(	0,0,0,
				0,0,1,
				0,1,0);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	

	glBegin(GL_TRIANGLES);
	for (unsigned int i = 0; i < f.size(); i++){
		glVertex3f(v[f[i].v1].x, v[f[i].v1].y, v[f[i].v1].z);
		glVertex3f(v[f[i].v2].x, v[f[i].v2].y, v[f[i].v2].z);
		glVertex3f(v[f[i].v3].x, v[f[i].v3].y, v[f[i].v3].z);
	}
	glEnd();
	
	glutSwapBuffers();
	glutPostRedisplay();
}

static void reshape (int w, int h){
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, w/h, near, 3 * far);
	glMatrixMode (GL_MODELVIEW);
}

static void saveImage(){
    unsigned char* image = (unsigned char*)malloc(sizeof(unsigned char) * 120 * 120);//make 120 a macro
    glReadPixels(0, 0, 120, 120, GL_RED, GL_UNSIGNED_BYTE, image);//make 120 a macro
    template_i _template = template_i(120,120,CV_8U,image,pitch,yaw);
    templates.push_back(_template);
    
    
	string path = "./templates/tpl-y "+to_string((int)yaw)+"-p "+to_string((int)pitch)+".pgm";
    cv::imwrite(path ,_template.temp);
}

void get_perspective_template(){
	display();
	unsigned char* image = (unsigned char*)malloc(sizeof(unsigned char) * 120 * 120);//make 120 a macro
	instant = template_i(120,120,CV_8U,image,pitch,yaw);
}

static void idle_function(){
	if(!done){
		yaw = yaw_step[p];
		pitch = pitch_step[q];
		display();
		cout<<"yaw: "<<yaw<<" pitch: "<<pitch<<endl;
		saveImage();
	
		if(q == pitch_step.size()-1){p++;q=0;}
		else q++;
		if(p == yaw_step.size())done = true;
	}
	else{
		idle_func_blocking.lock();
		get_perspective_template();
		idle_func_blocking.unlock();
		usleep(10000);
	}
}

void mouse(int button, int state, int x, int y){
	mouse_button = button;
	mouse_x = x;
	mouse_y = y;
}

void motion(int x, int y){
	switch(mouse_button){
	case GLUT_LEFT_BUTTON:
		if( x==mouse_x && y==mouse_y ) return;
		yaw -= (GLfloat)( x - mouse_x ) /10.0;
		pitch -= (GLfloat)( y - mouse_y ) /10.0;
		break;
	case GLUT_RIGHT_BUTTON:
		if( y==mouse_y ) return;
		if( y < mouse_y ) dist += (GLfloat)(mouse_y - y);
		else dist -= (GLfloat)(y - mouse_y);
		if( -dist < near ) dist = -near;
		if( -dist > far ) dist = -far;
		break;
	}
	mouse_x = x;
	mouse_y = y;
	glutPostRedisplay();
}

void init(){
	//dummy variables
	int a = 0;char **aa = 0;
	glutInit(&a, aa);
	loadobj("load.obj");
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize (120, 120);
	glutInitWindowPosition (100, 100);
	glutCreateWindow ("Template extractor");
	glutReshapeFunc(reshape);
	glutIdleFunc(idle_function);
	glutReshapeFunc(reshape) ;
	glutMotionFunc(motion);
	glutMouseFunc(mouse);
	glutMainLoop();
}

void init_opengl(){
	idle_func_blocking.lock();
	thread (init).detach();
}
