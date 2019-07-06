/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));


	// TODO: Fill in this function
	// For max iterations
	int cloudsize = cloud->points.size();

	for(int it = 0; it < maxIterations; it++){
		std::unordered_set<int> inliersCurrent = {};
		// Randomly sample subset and fit line

		int num1 = rand()%cloudsize;
		int num2 = rand()%cloudsize;
		while(num1 == num2){
			num2 = rand()%cloudsize;
		}

		pcl::PointXYZ point1, point2;
		point1 = cloud->points[num1];
		point2 = cloud->points[num2];

		// Ax+By+C = 0
		float A = point1.y - point2.y;
		float B = point2.x - point1.x;
		float C = point1.x * point2.y - point2.x * point1.y;

		// Measure distance between every point and fitted line
		float dist = 0;

		for(int i = 0; i<cloudsize;i++){
			dist = abs( cloud->points[i].x * A + cloud->points[i].y * B + C) / sqrt(pow(A,2)+pow(B,2));
			// If distance is smaller than threshold count it as inlier
			if(dist<distanceTol)
				inliersCurrent.insert(i);
		}

		if(inliersCurrent.size() > inliersResult.size()){
			std::cout << "Renew from " <<  inliersResult.size() << " to " << inliersCurrent.size() << std::endl;
			inliersResult = inliersCurrent;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));


	// TODO: Fill in this function
	// For max iterations
	int cloudsize = cloud->points.size();

	for(int it = 0; it < maxIterations; it++){
		std::unordered_set<int> inliersCurrent = {};
		// Randomly sample subset and fit line

		int num1 = rand()%cloudsize;
		int num2 = rand()%cloudsize;
		int num3 = rand()%cloudsize;
		while(num1 == num2 || num2 == num3 || num3 == num1){
			num2 = rand()%cloudsize;
			num3 = rand()%cloudsize;
		}

		pcl::PointXYZ point1, point2, point3;
		point1 = cloud->points[num1];
		point2 = cloud->points[num2];
		point3 = cloud->points[num3];


		std::vector<float> v1(3); // point1 to point2
		std::vector<float> v2(3); // point2 to point3

		v1[0] = point2.x - point1.x;
		v1[1] = point2.y - point1.y;
		v1[2] = point2.z - point1.z;

		v2[0] = point3.x - point2.x;
		v2[1] = point3.y - point2.y;
		v2[2] = point3.z - point2.z;


		// cross(v1, v2) = {i, j, k}
		// i(x-x1)+j(y-y1)+k(z-z1)= 0
		// Ax + By + Cz + D = 0
		float A = v1[1]*v2[2] - v1[2]*v2[1];
		float B = v1[2]*v2[0] - v1[0]*v2[2];
		float C = v1[0]*v2[1] - v1[1]*v2[0];
		float D = -1 * (A*point1.x+B*point1.y+C*point1.z);

		// Measure distance between every point and fitted line
		float dist = 0;

		for(int i = 0; i<cloudsize;i++){
			// ∣A∗x+B∗y+C∗z+D∣/sqrt(A^2  +B^2 +C^2 ).
			dist = abs(A * cloud->points[i].x  + B * cloud->points[i].y + C * cloud->points[i].z) / sqrt(pow(A,2)+pow(B,2)+pow(C,2));
			if(dist<distanceTol){
				// std::cout << "dist:" << dist << std::endl;
				inliersCurrent.insert(i);
				// std::cout << "\tinliers current size: " << inliersCurrent.size() << std::endl;
			}
		}
		// std::cout << "Done inliers current size: " << inliersCurrent.size() << std::endl;
		if(inliersCurrent.size() > inliersResult.size()){
			std::cout << "Renew from " <<  inliersResult.size() << " to " << inliersCurrent.size() << std::endl;
			inliersResult = inliersCurrent;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 1000, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
