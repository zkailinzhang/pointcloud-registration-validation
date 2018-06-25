#pragma once
#include "headers.h"
#include "MyPointCloud.h"

class UtilityPcl
{
public:
	UtilityPcl();
	~UtilityPcl();

	void showPC(CloudXYZPtr &input);

	float computeCosine(cv::Point3d & p1, cv::Point3d & p2, cv::Point3d & q1, cv::Point3d & q2);

	void kdsearch(CloudXYZPtr & input, cv::Mat& Normals, std::vector<float>& variance);

	bool write_ply(const std::string & filename, CloudXYZPtr &cloud, pcl::PolygonMesh& mesh, std::vector<cv::Vec3b>& gray_all);

public:
	MyPointCloud myPC;


};

