#include "stdafx.h"
#include "MyPointCloud.h"


MyPointCloud::MyPointCloud():
	Points(new CloudXYZ)
{
	//Points = new CloudXYZPtr;
}


MyPointCloud::~MyPointCloud()
{
}
void MyPointCloud::clear(void)
{
	Gray = cv::Mat();
	Normals = cv::Mat();
}

void MyPointCloud::init_points(int rows, CloudXYZPtr &input)
{
	
	size_t npts = rows;
	// In order to transform the data, we need to remove NaNs
	//Points->is_dense = input->is_dense;
	//Points->header = input->header;
	Points->width = static_cast<int> (npts);
	Points->height = 1;
	//Points->points.resize(npts);
	


	//if (!pcl_isfinite(cloud_in.points[i].x) ||!pcl_isfinite(cloud_in.points[i].y) ||!pcl_isfinite(cloud_in.points[i].z))
// 	for (size_t i = 0; i < npts; ++i)
// 	{
// 		points->points[i] = input->points[i];
// 	}
}

void MyPointCloud::init_color(int rows, int cols)
{
	Gray = cv::Mat::zeros(rows, cols, CV_8UC1);
	memset(Gray.data, 0, Gray.total()*Gray.channels()); //white
}

void MyPointCloud::init_normals(int rows, int cols)
{
	Normals = cv::Mat::zeros(rows, cols, CV_64FC1);
	// 	double * data = points.ptr<double>(0);
	// 	for (size_t i = 0; i < total; i++)
	// 	{
	// 		data[i] = std::numeric_limits<double>::quiet_NaN();
	// 	}
}




//读取文件filename的第n行，存到buf中，buf_len为buf的长度。
void MyPointCloud::read_data(const char *filetxt, CloudXYZPtr &input, int n, std::vector<int> &index)
{
	//ifstream in(filetxt);//打开文件。
	int i;
	double buf[6] = { 0.0 };
	int buf_len;

	cv::Mat tmp = cv::Mat::zeros(n, 6, CV_64FC1);;

	std::ifstream fin(filetxt, ios::out);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			double a;
			fin >> a;
			tmp.at<double>(i, j) = a;//将txt中数值写入到opencv矩阵中
		}
	}
	fin.close();

	int numvalid = index.size();
	Points->points.resize(numvalid);
	std::vector<int>::iterator iter = index.begin();
	for (int i = 0; iter != index.end(); ++iter, ++i)
	{
		for (int j = 0; j < 6; j++)
			Normals.at<double>(i, j) = tmp.at<double>(*iter, j);

		Points->points[i] = input->points[*iter];
		if (i==8000)
		{
			;
		}
	}

}

/*
	boost::filesystem::fstream ifs(file2);
	assert(ifs.is_open());
	std::stringstream sstr;
	while (ifs >> sstr.rdbuf());
	ifs.close();
	cv::Mat img_decode;
	std::string str_tmp = sstr.str();
	std::vector <  float > data(str_tmp.begin(), str_tmp.end());
	img_decode = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);*/
