#include "stdafx.h"
#include "UtilityPcl.h"


UtilityPcl::UtilityPcl()
{
}


UtilityPcl::~UtilityPcl()
{
}


void UtilityPcl::showPC(CloudXYZPtr &input) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ShowPC"));
	
	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ> scene_color_handler(input, 0, 255, 0);
	viewer->addPointCloud<PointXYZ>(input, scene_color_handler, "cloud");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud");

	while (!viewer->wasStopped())
		viewer->spinOnce(1000);

	viewer->close();

}

float UtilityPcl::computeCosine(cv::Point3d &p1, cv::Point3d &p2,
	cv::Point3d &q1, cv::Point3d &q2) {


	float fenmu = (q1.dot(p1) * q2.dot(p1)) + (q1.dot(p2)*q2.dot(p2));
	float fenzi = std::sqrt(  (std::pow(q1.dot(p1), 2) + std::pow(q1.dot(p2), 2) )
		* (std::pow(q2.dot(p1), 2) + std::pow(q2.dot(p2), 2))  );


	return (float)fenmu / fenzi;
}



void UtilityPcl::kdsearch(CloudXYZPtr &input, cv::Mat& Normals, std::vector<float> &variance){
	


	CloudXYZPtr kp_pc(new CloudXYZ);
	CloudXYZPtr nn_pc(new CloudXYZ);

	*kp_pc = *nn_pc = *input;

	PCL_INFO("   : %d Points \n", kp_pc->points.size());
	PCL_INFO("   : %d Points \n", nn_pc->points.size());

	int num_nn = 20;
	float radius_nn = 3.;
	int td_num = 30;

	pcl::search::KdTree<PointXYZ> match_search;

	//20170531 zkl  is not typename
	//	typename  pcl::L2_SQR d;
	//	pcl::KdTreeFLANN<DescriptorType,flann::L2_Simple<float>> match_search;
	match_search.setInputCloud(kp_pc);     
												 
	std::map<int, cv::Point3d> kp_points;
	std::map<int, std::vector<cv::Point3d> > nn_points;

	for (size_t j = 0; j < nn_pc->size(); ++j) 
	{
		std::vector<int> neigh_indices(num_nn);           
		std::vector<float> neigh_sqr_dists(num_nn);

		//int found_neighs = match_search.nearestKSearch(nn_pc->at(j), num_nn, neigh_indices, neigh_sqr_dists);	
		int found_neighs = match_search.radiusSearch(nn_pc->at(j), radius_nn, neigh_indices, neigh_sqr_dists);
// 		if (found_neighs == num_nn)
// 		{
		//if (found_neighs < 30)
		//{
		//	variance.push_back(0.);
		//	continue;
		//}   
		//cout << " " << found_neighs << " ";
			cv::Point3d kp(cv::Point3d(nn_pc->at(j).x, nn_pc->at(j).y, nn_pc->at(j).z));

			kp_points.insert(std::map<int, cv::Point3d>::value_type(j, kp));

			cv::Mat imgDev;
			if (found_neighs>td_num)
				 imgDev= cv::Mat::zeros(td_num -1, 1, CV_32FC1);
			else
				 imgDev= cv::Mat::zeros(found_neighs - 1, 1, CV_32FC1);
			//k=0,itself
			
			for (int k = 1; k < found_neighs; k++)
			{
				cv::Point3d nn(cv::Point3d(kp_pc->at(neigh_indices[k]).x, kp_pc->at(neigh_indices[k]).y, kp_pc->at(neigh_indices[k]).z));
				nn_points[j].push_back(nn);


				//compute cosine for every nn_points
				cv::Point3d p1(cv::Point3d(Normals.at<double>(j, 0), Normals.at<double>(j, 1), Normals.at<double>(j, 2)));
				cv::Point3d p2(cv::Point3d(Normals.at<double>(j, 3), Normals.at<double>(j, 4), Normals.at<double>(j, 5)));

				cv::Point3d q1(cv::Point3d(Normals.at<double>(neigh_indices[k], 0), Normals.at<double>(neigh_indices[k], 1),
					Normals.at<double>(neigh_indices[k], 2)));
				cv::Point3d q2(cv::Point3d(Normals.at<double>(neigh_indices[k], 3), Normals.at<double>(neigh_indices[k], 4),
					Normals.at<double>(neigh_indices[k], 5)));

				float cosine = computeCosine(p1, p2, q1, q2);

				imgDev.at<float>(k-1, 0) = cosine;

				if (found_neighs >td_num)
				    break;

			}
		
			//,then compute convariance
			cv::Mat tmp_m, tmp_sd;
			cv::meanStdDev(imgDev, tmp_m, tmp_sd);
			double sd = tmp_sd.at<double>(0, 0);
			variance.push_back(sd*sd);

		

	}

}



bool UtilityPcl::write_ply(const std::string & filename, CloudXYZPtr &cloud, pcl::PolygonMesh& mesh,std::vector<cv::Vec3b>& gray_all) {
	
	if (! (cloud->width == gray_all.size() ))
	{
		return false;
	}

	std::ofstream outfile;
	std::ios::openmode mode = std::ios::out | std::ios::trunc ;
	outfile.open(filename.c_str(), mode);
	if (!outfile.is_open())
	{
		return false;
	}
	int total = static_cast<int>(cloud->size());
	int facesize = mesh.polygons.size();

	const char * format_header =  "ascii 1.0" ;
	outfile << "ply" << std::endl
		<< "format " << format_header << std::endl
		<< "comment VCGLIB generated" << std::endl
		<< "element vertex " << total << std::endl
		<< "property float x" << std::endl
		<< "property float y" << std::endl
		<< "property float z" << std::endl
		<< "property uchar red" << std::endl
		<< "property uchar green" << std::endl
		<< "property uchar blue" << std::endl;

	outfile << "element face "<< facesize << std::endl
		<< "property list uchar int vertex_indices" << std::endl
		<< "end_header" << std::endl;

	//save pointxyz nx ny nz
	for (int i=0;i<total;i++)
	{
		outfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z;
		outfile << " " <<(int)(gray_all[i]).val[0] << " " << (int)(gray_all[i].val[1]) << " " << (int)(gray_all[i].val[2]);
		outfile << std::endl;	
	}


	//save face mesh
	
	std::vector< pcl::Vertices> face_vec = mesh.polygons;
	for (int i=0;i<facesize;i++)
	{	 	
	 	pcl::Vertices bbb = face_vec[i];
		outfile <<3<< " " << bbb.vertices[0]<<" " << bbb.vertices[1]<<  " " << bbb.vertices[2];
		outfile << std::endl;
	}





	outfile.close();
	std::cerr << "[write_ply] Saved " << total << " points (" << filename << ")" << std::endl;

	return true;




}