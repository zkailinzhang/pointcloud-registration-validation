#pragma once

#include <pcl/console/parse.h>

#include <pcl/io/ply_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/2d/convolution.h>
#include <pcl/2d/kernel.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp_nl.h>

//#include <pcl/registration/incremental_registration.h> pcl18
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

#include <pcl/visualization/registration_visualizer.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/default_convergence_criteria.h>


#include <pcl/common/eigen.h>
//#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

typedef pcl::PointXYZ              PointXYZ;
typedef pcl::PointCloud <PointXYZ> CloudXYZ;
typedef CloudXYZ::Ptr              CloudXYZPtr;
typedef CloudXYZ::ConstPtr         CloudXYZConstPtr;
typedef pcl::Normal NormalType;
typedef pcl::PointNormal              PointNormal;
typedef pcl::PointCloud <PointNormal> CloudNormal;
typedef CloudNormal::Ptr              CloudNormalPtr;
typedef pcl::PointXYZRGBA PointType;
typedef pcl::ReferenceFrame RFType;
