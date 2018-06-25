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

double UtilityPcl::computeVariance(cv::Point3d &p1, cv::Point3d &p2,
	cv::Point3d &q1, cv::Point3d &q2) {


	double fenmu = (q1.dot(p1) * q2.dot(p1)) + (q1.dot(p2)*q1.dot(p2));
	double fenzi = std::sqrt((std::pow(q1.dot(p1), 2), std::pow(q1.dot(p2), 2))
		* (std::pow(q2.dot(p1), 2), std::pow(q1.dot(p2), 2)));


	return (double)fenmu / fenzi;
}



void UtilityPcl::kdsearch(CloudXYZPtr &input,std::vector<double> &variance){
	


	CloudXYZPtr kp_pc(new CloudXYZ);
	CloudXYZPtr nn_pc(new CloudXYZ);

	*kp_pc = *nn_pc = *input;

	PCL_INFO("  model sift : %d Points \n", kp_pc->points.size());
	PCL_INFO("  scene sift : %d Points \n", nn_pc->points.size());

	

	int num_nn = 20;
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

		int found_neighs = match_search.nearestKSearch(nn_pc->at(j), num_nn, neigh_indices, neigh_sqr_dists);	

		if (found_neighs == num_nn)
		{
			//第二个参数 ？  两个点 及距离  。索引是在搜素空间搜到最近邻的那个点的索引值
			/**  Index of the matching (target) point. Set to -1 if no correspondence found. */
			pcl::Correspondence corr(static_cast<int> (j), neigh_indices[0], neigh_sqr_dists[0]);
			//float d = pcl::L2_Norm(scene_descriptors->at(j), model_descriptors->at(corr.index_match), 361);
			// 			float norm = 0.0;
			// 			for (int m = 0; m < scene_kp_xyz->at(j).descriptorSize() ; ++m)
			// 			{
			// 				float diff = scene_descriptors->at(j).descriptor[m] - model_descriptors->at(corr.index_match).descriptor[m];
			// 				norm += diff*diff;
			// 			}
			// 
			// 			std::cout << "corr.index_match corr.index_query neigh_indices[0], static_cast<int> (j),"
			// 				<< corr.index_match <<" "<< corr.index_query << " " << neigh_indices[0] << " " << static_cast<int> (j) << endl;
			// 			std::cout << norm<< endl;

			cv::Point3d kp(cv::Point3d(nn_pc->at(j).x, nn_pc->at(j).y, nn_pc->at(j).z));

			kp_points.insert(std::map<int, cv::Point3d>::value_type(j, kp));

			cv::Mat imgDev(found_neighs, 1, CV_64FC1, cv::Scalar(0));

			for (int k = 0; k < found_neighs; k++)
			{
				cv::Point3d nn(cv::Point3d(kp_pc->at(neigh_indices[k]).x, kp_pc->at(neigh_indices[k]).y, kp_pc->at(neigh_indices[k]).z));
				nn_points[j].push_back(nn);


				//compute cosine for every nn_points


				cv::Point3d p1(cv::Point3d(myPC.Normals.at<double>(j, 0), myPC.Normals.at<double>(j, 1), myPC.Normals.at<double>(j, 2)));
				cv::Point3d p2(cv::Point3d(myPC.Normals.at<double>(j, 3), myPC.Normals.at<double>(j, 4), myPC.Normals.at<double>(j, 5)));

				cv::Point3d q1(cv::Point3d(myPC.Normals.at<double>(neigh_indices[k], 0), myPC.Normals.at<double>(neigh_indices[k], 1),
					myPC.Normals.at<double>(neigh_indices[k], 2)));
				cv::Point3d q2(cv::Point3d(myPC.Normals.at<double>(neigh_indices[k], 3), myPC.Normals.at<double>(neigh_indices[k], 4),
					myPC.Normals.at<double>(neigh_indices[k], 5)));

				double cosine = computeVariance(p1, p2, q1, q2);

				imgDev.at<double>(k, 0) = cosine;



			}
		
			//,then compute convariance
			cv::Mat tmp_m, tmp_sd;
			meanStdDev(imgDev, tmp_m, tmp_sd);
			
			double sd = tmp_sd.at<double>(0, 0);

			variance.push_back((double)sd*sd);


		}
	}
}




/*

void scan3d::reconstruct_model_patch_center(Pointcloud & pointcloud, CalibrationData const& calib,
	cv::Mat const& pattern_image, cv::Mat const& min_max_image, cv::Mat const& color_image,
	cv::Size const& projector_size, cv::Mat &proj_image, int threshold, double max_dist)
{
	if (!pattern_image.data || pattern_image.type() != CV_64FC2)
	{   //pattern not correctly decoded
		std::cerr << "[reconstruct_model] ERROR invalid pattern_image\n";
		return;
	}
	if (!min_max_image.data || min_max_image.type() != CV_8UC2)
	{   //pattern not correctly decoded
		std::cerr << "[reconstruct_model] ERROR invalid min_max_image\n";
		return;
	}
	if (color_image.data && color_image.type() != CV_8UC3)
	{   //not standard RGB image
		std::cerr << "[reconstruct_model] ERROR invalid color_image\n";
		return;
	}
	if (!calib.is_valid())
	{   //invalid calibration
		return;
	}

	//parameters
	//const unsigned threshold = config.value("main/shadow_threshold", 70).toUInt();
	//const double   max_dist  = config.value("main/max_dist_threshold", 40).toDouble();
	//const bool     remove_background = config.value("main/remove_background", true).toBool();
	//const double   plane_dist = config.value("main/plane_dist", 100.0).toDouble();
	double plane_dist = 100.0;

	/* background removal
	cv::Point2i plane_coord[3];
	plane_coord[0] = cv::Point2i(config.value("background_plane/x1").toUInt(), config.value("background_plane/y1").toUInt());
	plane_coord[1] = cv::Point2i(config.value("background_plane/x2").toUInt(), config.value("background_plane/y2").toUInt());
	plane_coord[2] = cv::Point2i(config.value("background_plane/x3").toUInt(), config.value("background_plane/y3").toUInt());

	if (plane_coord[0].x<=0 || plane_coord[0].x>=pattern_local.cols
	|| plane_coord[0].y<=0 || plane_coord[0].y>=pattern_local.rows)
	{
	plane_coord[0] = cv::Point2i(50, 50);
	config.setValue("background_plane/x1", plane_coord[0].x);
	config.setValue("background_plane/y1", plane_coord[0].y);
	}
	if (plane_coord[1].x<=0 || plane_coord[1].x>=pattern_local.cols
	|| plane_coord[1].y<=0 || plane_coord[1].y>=pattern_local.rows)
	{
	plane_coord[1] = cv::Point2i(50, pattern_local.rows-50);
	config.setValue("background_plane/x2", plane_coord[1].x);
	config.setValue("background_plane/y2", plane_coord[1].y);
	}
	if (plane_coord[2].x<=0 || plane_coord[2].x>=pattern_local.cols
	|| plane_coord[2].y<=0 || plane_coord[2].y>=pattern_local.rows)
	{
	plane_coord[2] = cv::Point2i(pattern_local.cols-50, 50);
	config.setValue("background_plane/x3", plane_coord[2].x);
	config.setValue("background_plane/y3", plane_coord[2].y);
	}
	*/

	//init point cloud  点云为投影分辨率
	int scale_factor_x = 1;
	int scale_factor_y = (projector_size.width>projector_size.height ? 1 : 2); //XXX HACK: preserve regular aspect ratio XXX HACK
	int out_cols = projector_size.width / scale_factor_x;
	int out_rows = projector_size.height / scale_factor_y;
	pointcloud.clear();
	pointcloud.init_points(out_rows, out_cols);
	pointcloud.init_color(out_rows, out_cols);

	//progress


	//take 3 points in back plane
	/*cv::Mat plane;
	if (remove_background)
	{
	cv::Point3d p[3];
	for (unsigned i=0; i<3;i++)
	{
	for (unsigned j=0;
	j<10 && (
	INVALID(pattern_local.at<cv::vec2d>(plane_coord[i].y, plane_coord[i].x)[0])
	|| INVALID(pattern_local.at<cv::vec2d>(plane_coord[i].y, plane_coord[i].x)[1])); j++)
	{
	plane_coord[i].x += 1.f;
	}
	const cv::vec2d & pattern = pattern_local.at<cv::vec2d>(plane_coord[i].y, plane_coord[i].x);

	const double col = pattern[0];
	const double row = pattern[1];

	if (projector_size.width<=static_cast<int>(col) || projector_size.height<=static_cast<int>(row))
	{   //abort
	continue;
	}

	//shoot a ray through the image: u=\lambda*v + q
	cv::Point3d u1 = camera.to_world_coord(plane_coord[i].x, plane_coord[i].y);
	cv::Point3d v1 = camera.world_ray(plane_coord[i].x, plane_coord[i].y);

	//shoot a ray through the projector: u=\lambda*v + q
	cv::Point3d u2 = projector.to_world_coord(col, row);
	cv::Point3d v2 = projector.world_ray(col, row);

	//compute ray-ray approximate intersection
	double distance = 0.0;
	p[i] = geometry::approximate_ray_intersection(v1, u1, v2, u2, &distance);
	std::cout << "Plane point " << i << " distance " << distance << std::endl;
	}
	plane = geometry::get_plane(p[0], p[1], p[2]);
	if (cv::Mat(plane.rowRange(0,3).t()*cv::Mat(cv::Point3d(p[0].x, p[0].y, p[0].z-1.0)) + plane.at<double>(3,0)).at<double>(0,0)
	<0.0)
	{
	plane = -1.0*plane;
	}
	std::cout << "Background plane: " << plane << std::endl;
	}
	*/

	//candidate points  相机投影多对一？？？？
	std::map<unsigned, cv::Point2d> proj_points;
	std::map<unsigned, std::vector<cv::Point2d> > cam_points;

	proj_image = cv::Mat::zeros(out_rows, out_cols, CV_8UC3);
	//zkl0401
	unsigned good = 0;
	unsigned bad = 0;
	unsigned invalid = 0;
	unsigned repeated = 0;
	std::cout << "Reconstruction in progress: collecting points" << std::endl;
	for (int h = 0; h<pattern_image.rows; h++)
	{


		register const cv::Vec2d * curr_pattern_row = pattern_image.ptr<cv::Vec2d>(h);
		register const cv::Vec2b * min_max_row = min_max_image.ptr<cv::Vec2b>(h);
		for (register int w = 0; w<pattern_image.cols; w++)
		{
			const cv::Vec2d & pattern = curr_pattern_row[w];
			const cv::Vec2b & min_max = min_max_row[w];

			if (sl::INVALID(pattern)
				|| pattern[0]<0.0 || pattern[0] >= projector_size.width || pattern[1]<0.0 || pattern[1] >= projector_size.height
				|| (min_max[1] - min_max[0])<static_cast<int>(threshold))
			{   //skip
				continue;
			}

			//ok
			cv::Point2d proj_point(pattern[0] / scale_factor_x, pattern[1] / scale_factor_y);
			unsigned index = static_cast<unsigned>(proj_point.y)*out_cols + static_cast<unsigned>(proj_point.x);

			//zkl 20170925 correspences
			proj_points.insert(std::map<unsigned, cv::Point2d>::value_type(index, proj_point));
			//std::mapmap<int, string>::value_type   std::piar

			cam_points[index].push_back(cv::Point2d(w, h));

			proj_image.at<cv::Vec3b>(static_cast<unsigned>(proj_point.y), static_cast<unsigned>(proj_point.x)) = color_image.at<cv::Vec3b>(h, w);
		}
	}

	//zkl 0926
	cv::imwrite("proj_image_wa.png", proj_image);

	//     if (progress)
	//     {
	//         progress->setValue(pattern_image.rows);
	//     }

	// 转置
	cv::Mat Rt = calib.R.t();

	//     if (progress)
	//     {
	//         progress->setMaximum(proj_points.size());
	//     }

	// 关联map容器， iter1->first是键 ->second是值
	std::map<unsigned, cv::Point2d>::iterator iter1 = (proj_points.begin());

	unsigned n = 0;

	//zkl0927 save raw img proj correspondence
	cv::Mat cam_raw(proj_points.size(), 1, CV_64FC2);
	cv::Mat proj_raw(proj_points.size(), 1, CV_64FC2);

	// pointcloud.projuv = cv::Mat(proj_points.size(),1,CV_64FC2);
	// pointcloud.camuv = cv::Mat(proj_points.size(),1,CV_64FC2);

	//std::map<unsigned, cv::Point2f> end = (proj_points.end());
	while (iter1 != proj_points.end()) // _isnan((*iter1).first)  (iter1)->first 他的值就是0   iter1.hasNext()
	{
		n++;
		//std::cout << "Reconstruction in progress: %1 good points/%2 bad points" << (good) << " "<<(bad) << std::endl;

		unsigned  index = (*iter1).first;
		const cv::Point2d & proj_point = (*iter1).second;
		//unsafe
		const std::vector<cv::Point2d> & cam_point_list = (*cam_points.find(index)).second;

		//safe
		//std::map<unsigned, std::vector<cv::Point2f>>::iterator it = cam_points.find(index);
		//if (it != cam_points.end()) 
		//const std::vector<cv::Point2f> & cam_point_list = it->second;



		const unsigned count = static_cast<int>(cam_point_list.size());

		if (!count)
		{   //empty list
			continue;
		}

		//center average
		cv::Point2d sum(0.0, 0.0), sum2(0.0, 0.0);

		for (std::vector<cv::Point2d>::const_iterator iter2 = cam_point_list.begin(); iter2 != cam_point_list.end(); iter2++)
		{
			sum.x += iter2->x;
			sum.y += iter2->y;
			sum2.x += (iter2->x)*(iter2->x);//没用
			sum2.y += (iter2->y)*(iter2->y);
		}
		cv::Point2d cam(sum.x / count, sum.y / count);
		cv::Point2d proj(proj_point.x*scale_factor_x, proj_point.y*scale_factor_y);


		//zkl 0927
		cam_raw.at<cv::Vec2d>(n - 1, 0)[0] = (double)sum.x / count;
		cam_raw.at<cv::Vec2d>(n - 1, 0)[1] = (double)sum.y / count;
		proj_raw.at<cv::Vec2d>(n - 1, 0)[0] = (double)proj.x;
		proj_raw.at<cv::Vec2d>(n - 1, 0)[1] = (double)proj.y;



		//triangulate
		double distance = max_dist;  //quality meassure
		cv::Point3d p;          //reconstructed point
		triangulate_stereo(calib.cam_K, calib.cam_kc, calib.proj_K, calib.proj_kc, Rt, calib.T, cam, proj, p, &distance);

		if (distance < max_dist)
		{   //good point

			//evaluate the plane
			double d = plane_dist + 1;
			/*if (remove_background)
			{
			d = cv::Mat(plane.rowRange(0,3).t()*cv::Mat(p) + plane.at<double>(3,0)).at<double>(0,0);
			}*/
			//
			if ((d > plane_dist)
				//wa
				//&& (p.x <= 110.0) && (p.x >= -20.0) && (p.y <= 50) && (p.y >= -110.0) 
				//&& (p.z <= 360.0) && (p.z >= 290.0)

				//right
				&& (p.x <= 400.0) && (p.x >= -600.0) && (p.y <= 240.0) && (p.y >= -550.0)
				&& (p.z <= 1750.0) && (p.z >= 1390.0)
				//left 10.10 before
				//	&& (p.x <= 200.0) && (p.x >= -700.0) && (p.y <= 140.0) && (p.y >= -600.0)
				//	&& (p.z <= 1650.0) && (p.z >= 1350.0)
				)
			{   //object point, keep
				good++;
				if (good == 68715)
				{
					good++;
					good--;
				}
				//std::cout << good << std::endl;
				cv::Vec3d & cloud_point = pointcloud.points.at<cv::Vec3d>(proj_point.y, proj_point.x);
				cloud_point[0] = p.x;
				cloud_point[1] = p.y;
				cloud_point[2] = p.z;

				//zkl 20170927

				(pointcloud.camPoints2).push_back(cam);
				(pointcloud.projPoints2).push_back(proj);
				// undifine cant use,no memrory
				// pointcloud.camuv.at<cv::vec2d>(good-1, 0)[0] = static_cast<double>(cam.x);
				// pointcloud.camuv.at<cv::vec2d>(good-1, 0)[1] = static_cast<double>(cam.y);
				// pointcloud.projuv.at<cv::vec2d>(good-1, 0)[0] = static_cast<double>(proj.x);
				// pointcloud.projuv.at<cv::vec2d>(good-1, 0)[1] = static_cast<double>(proj.y);


				if (color_image.data)
				{
					const cv::Vec3b & vec = color_image.at<cv::Vec3b>(static_cast<unsigned>(cam.y), static_cast<unsigned>(cam.x));
					cv::Vec3b & cloud_color = pointcloud.colors.at<cv::Vec3b>(proj_point.y, proj_point.x);
					cloud_color[0] = vec[0];
					cloud_color[1] = vec[1];
					cloud_color[2] = vec[2];
				}
			}
		}
		else
		{   //skip
			bad++;
			if (bad == 82400)
			{
				bad++;
				bad--;
			}
			//std::cout << " d = " << distance << std::endl;
		}

		iter1++;
	}   //while


		//zkl0927
		//    pointcloud.camuv = new cv::Mat(camPoints2.size(),1,CV_64FC2);
		//    pointcloud.projuv =new  cv::Mat(projPoints2.size(),1,CV_64FC2);
		// pointcloud.camuv = &cv::Mat(camPoints2);
		//  pointcloud.projuv =& cv::Mat(projPoints2);
		//  std::vector<cv::Point2f> projvec22 =cv::Mat_<cv::Point2f>(* pointcloud.camuv);
		//  std::vector<cv::Point2f> projvec23 =cv::Mat_<cv::Point2f>(* pointcloud.projuv);



	std::cout << "Reconstructed points [patch center]: " << good << " (" << bad << " skipped, " << invalid << " invalid) " << std::endl
		<< " - repeated points: " << repeated << " (ignored) " << std::endl;
}



void  PCL_COM::demean_standardization(cv::Mat &pointcloud,
	int &num, cv::Mat &cloud_out) {

	cv::Mat tmp_m, tmp_sd;

	cv::meanStdDev(pointcloud, tmp_m, tmp_sd);
	//demean_norm_Mat*srt =dst  3*4 4*n 3*n;n*1 c3
	// 	int h = mat.size(), w = mat[0].size();
	// 	double sum{ 0. }, sqsum{ 0. };
	// 
	// 	for (int y = 0; y < h; ++y) {
	// 		for (int x = 0; x < w; ++x) {
	// 			double v = static_cast<double>(mat[y][x]);
	// 			sum += v;
	// 			sqsum += v * v;
	// 		}
	// 	}
	// 
	// 	double scale = 1. / (h * w);
	// 	*mean = sum * scale;
	// 	*variance = std::max(sqsum*scale - (*mean)*(*mean), 0.);
	// 	*stddev = std::sqrt(*variance);

	A_StdDev(0, 0) = 1 / tmp_sd.at<double>(0, 0);
	A_StdDev(0, 3) = -tmp_m.at<double>(0, 0) / tmp_sd.at<double>(0, 0);

	A_StdDev(1, 1) = 1 / tmp_sd.at<double>(1, 0);
	A_StdDev(1, 3) = -tmp_m.at<double>(1, 0) / tmp_sd.at<double>(1, 0);

	A_StdDev(2, 2) = 1 / tmp_sd.at<double>(2, 0);
	A_StdDev(2, 3) = -tmp_m.at<double>(2, 0) / tmp_sd.at<double>(2, 0);

	A_StdDev(3, 3) = 1;


	for (int k = 0; k < num; k++)
	{
		for (int i = 0; i < 3; i++)
		{
			cloud_out.at<cv::Vec3d>(k, 0)[i] =// pointcloud.at<cv::Vec3d>(k, 0)[i];
				(pointcloud.at<cv::Vec3d>(k, 0)[i] - tmp_m.at<double>(i, 0)) / tmp_sd.at<double>(i, 0);

		}
	}




}
