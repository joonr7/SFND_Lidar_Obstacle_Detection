/* \author Aaron Brown */
/* \editor Joonyoung */

#include "ransac3d.h"

// template<typename PointT>
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
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
		// std::cout << "Random points #: " << num1 << ", " << num2 << ", " << num3 << std::endl;
		pcl::PointXYZI point1, point2, point3;
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
		// Plane: i(x-x1)+j(y-y1)+k(z-z1)= 0
		// Plane: Ax + By + Cz + D = 0
		float A = v1[1]*v2[2] - v1[2]*v2[1];
		float B = v1[2]*v2[0] - v1[0]*v2[2];
		float C = v1[0]*v2[1] - v1[1]*v2[0];
		float D = -1 * (A*point1.x+B*point1.y+C*point1.z);

		// Measure distance between every point and fitted line
		float dist = 0;

		for(int i = 0; i<cloudsize; i++){
			// distance between a plane(Ax+By+Cz+D = 0) and a point(x,y,z):  ∣A∗x+B∗y+C∗z+D∣/sqrt(A^2  +B^2 +C^2 ).
			dist = abs(A * cloud->points[i].x  + B * cloud->points[i].y + C * cloud->points[i].z + D) / sqrt(pow(A,2)+pow(B,2)+pow(C,2));

			if(dist<distanceTol){
				inliersCurrent.insert(i);
			}
		}

		if(inliersCurrent.size() > inliersResult.size()){
			std::cout << "Renew from " <<  inliersResult.size() << " to " << inliersCurrent.size() << std::endl;
			inliersResult = inliersCurrent;
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}
