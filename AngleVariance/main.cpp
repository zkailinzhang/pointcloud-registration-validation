// AngleVariance.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "headers.h"

#include <fstream>
#include <iostream>
#include "MyPointCloud.h"
#include "UtilityPcl.h"


#include <vector>
#include <algorithm>

#include <pcl/visualization/vtk.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyDataReader.h>

using namespace std;

template<typename Tpc, typename Tp>
double computeCloudResolution(Tpc &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<Tp> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))  //查询点云中 x坐标
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

int main()
{
	std::string filename = ".\\rect\\mw_mode.ply";
	std::string filetxt = ".\\rect\\pg_dir.txt";
	std::string filehist = ".\\hist.txt";
	std::string  saveply = "test_mw_mode_2.ply";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudgray(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPLYFile(saveply, *cloudgray);
	
	pcl::PolygonMesh mesh;
	pcl::PLYReader ply;
	//The following three all ok,read xyzrgb face,and show rgb ok
	//ply.read(saveply, mesh);
	//pcl::io::loadPolygonFilePLY(saveply, mesh);
	pcl::io::loadPLYFile(saveply, mesh);
	

// 	std::vector< pcl::Vertices> aaa = mesh.polygons;
// 	pcl::Vertices bbb = aaa[0];
// 	cout << bbb.vertices[0]<<" " << bbb.vertices[1]<<  " " << bbb.vertices[2];


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowPC"));
	viewer->setBackgroundColor(0.3, 0.3, 0.3);
	//viewer->setShapeRenderingProperties()

	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb();
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);
	//pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> scene_color_handler(cloudgray);
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloudgray, "cloud");
	viewer->addPolygonMesh(mesh,"cloud");	
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, 1, "cloud");

	while (!viewer->wasStopped())
		viewer->spinOnce(1000);

	viewer->close();


	CloudXYZPtr cloud(new CloudXYZ);
	CloudXYZPtr cloud_filtered(new CloudXYZ);

	pcl::io::loadPLYFile(filename, *cloud);
	double reso = computeCloudResolution<CloudXYZPtr, pcl::PointXYZ>(cloud);
	cout << "分辨率 " << reso << endl;
	int number = cloud->size();

	std::ifstream fin(filetxt, ios::out); 
	
	int num_valid = 0;
	std::vector<int> index;
	for (int i = 0; i < number *6; i++)
	{
		double a;
		fin >> a;

		//getline(fin,)
		if ((!(i % 6)) && (a != 0.0))
		{
			num_valid++;
			index.push_back(i / 6);
			for (int k = 0; k < 5; k++, i++)
				fin >> a;
		}
	}
	std::cout << num_valid;
	fin.close(); 

	MyPointCloud mypc;
	mypc.init_normals(num_valid, 6);
	mypc.init_color(num_valid, 1);
	mypc.init_points(num_valid,cloud);

	//fetch valid point
	mypc.read_data(filetxt.data(), cloud, number, index);

	
	//show 
	UtilityPcl comPcl;
	comPcl.showPC(mypc.Points);

	std::vector<float> variance;
	comPcl.kdsearch(mypc.Points, mypc.Normals, variance);


	auto biggest = std::max_element(std::begin(variance), std::end(variance));
	std::cout << "Max element is " << *biggest<< std::endl;

	auto smallest = std::min_element(std::begin(variance), std::end(variance));
	std::cout << "min element is " << *smallest<< std::endl;
	
	cv::Mat gray_tmp = mypc.Gray;
	float tmp_mean(0.0);
	for (int i = 0; i < num_valid; i++)
		tmp_mean += variance[i] ;

	//double tmp_mean2 = cv::mean(cv::Mat(variance));
	int hist[256] = { 0 };
	for (int i = 0; i < num_valid; i++)
	{
		uchar tmp_ = (variance[i] - (*smallest)) * 255 / (float)((*biggest) - (*smallest)); //tmp_mean;//		
		for (int i =0;i<256;i++)
			if ((int)tmp_ == i)
			{
				hist[i]++; 
				break;
			}
		mypc.Gray.at<uchar>(i, 0) = tmp_;

	}
	std::ofstream outfile;
	std::ios::openmode mode = std::ios::out | std::ios::trunc;
	outfile.open(filehist.c_str(), mode);
	for (int i = 0; i < 256; i++)
	{
		outfile << hist[i] << "  ";
		cout << hist[i] << "  ";
	}
	outfile.close();
	//show hist
	cv::Mat hist_img = cv::Mat::zeros(cv::Size( 256*10, 3000), CV_8UC3);
	
	for (int i=0;i<256;i++)
	{
		//cv::line(hist_img,cv::Point2i(i, hist[i]),cv::Point2i(i) )
		cv::rectangle(hist_img, cv::Point2f(i, 3000 - hist[i]), cv::Point2f(i + 10, 3000), cv::Scalar::all(255), CV_FILLED);
	}
	cv::namedWindow("hist", 0);
	cv::imshow("hist", hist_img);
	cv::waitKey();
	
	
	
	
	std::vector< cv::Vec3b > gray_all;
	//gray_all.resize(number);
	
	for (int i = 0, k = 0; i < number; i++)
	{
		if (i == index[k])
		{
			gray_all.push_back(cv::Vec3b(0, mypc.Gray.at<uchar>(k++, 0), 0));
			if ((int)(gray_all[i].val[1]) >180)
			   cout << " " << (int)(gray_all[i]).val[0] << " " << (int)(gray_all[i].val[1]) << " " << (int)(gray_all[i].val[2]) << endl;
		}
		else
		    gray_all.push_back(cv::Vec3b(0, 0,255));

	}
	


	cv::FileStorage fs("gray.xml", cv::FileStorage::WRITE);
	fs <<"Mat"<< mypc.Gray;
	fs<<"x"<< gray_all;
	fs.release();

	
	comPcl.write_ply(saveply, cloud, mesh,gray_all);


	cv::Point3d one;
	pcl::PointXYZ two= cloud->at(0);






    return 0;
}

