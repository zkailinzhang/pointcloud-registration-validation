#pragma once
#include "headers.h"



class MyPointCloud
{
public:
	MyPointCloud();
	~MyPointCloud();


public:
	void clear(void);
	void init_points(int rows, CloudXYZPtr &input);
	void init_color(int rows, int cols);
	void init_normals(int rows, int cols);

	void read_data(const char *filetxt, CloudXYZPtr &input, int n, std::vector<int> &index);

	
	//cv::Mat points; //N
	CloudXYZPtr Points;
	cv::Mat Gray; //N*1
	cv::Mat Normals; //N*6



};

