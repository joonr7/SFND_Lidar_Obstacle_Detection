// PCL lib Functions for processing point clouds

#include "processPointClouds.h"



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new typename pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Voxel Filter
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    // Region of Interest
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new typename pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cbf(true);
    cbf.setInputCloud(cloud_filtered);
    cbf.setMin(minPoint);
    cbf.setMax(maxPoint);
    cbf.filter(*cloud_cropped);

    // Roof remove (ego roof)
    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloud_cropped);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
      inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr objectCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);

    /////////////////// start //////////////////

    // // Create the filtering object
    // typename pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (inliers);
    //
    // extract.setNegative (false);
    // extract.filter (*planeCloud);
    // extract.setNegative (true);
    // extract.filter (*objectCloud);

    ///////////////// end /////////////////////

    // or
    ///////////////// start //////////////////
    for(int index: inliers->indices){
      planeCloud->points.push_back(cloud->points[index]);
    }

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*objectCloud);

    /////////////// end /////////////////////


    std::cerr << "PointCloud representing the planar component: " << planeCloud->width * planeCloud->height << " data points." << std::endl;
    std::cerr << "PointCloud representing the object component: " << objectCloud->width * objectCloud->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(objectCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	  // pcl::PointIndices::Ptr inliers;

    // Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr pcl_inliers (new pcl::PointIndices());

    // Create the segmentation object
    // Without PCL Segmentation //
    std::unordered_set<int> inliers = Ransac3D (cloud, maxIterations, distanceThreshold);

    std::cout << "Point cloud size after RANSAC: " << cloud->points.size() << std::endl;
    std::cout << "inliers size after RANSAC: " << inliers.size() << std::endl;

    for(std::unordered_set<int>::iterator itr = inliers.begin(); itr != inliers.end(); itr++)
        pcl_inliers->indices.push_back(*itr);

    // typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new typename pcl::PointCloud<PointT>());
  	// typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());
    //
    // for(int index = 0; index < cloud->points.size(); index++)
  	// {
  	// 	PointT point = cloud->points[index];
  	// 	if(inliers.count(index))
  	// 		cloudInliers->points.push_back(point);
  	// 	else
  	// 		cloudOutliers->points.push_back(point);
  	// }
    // std::cout << "cloudInliers->points size after RANSAC: " << cloudInliers->points.size() << std::endl;
    // std::cout << "cloudOutliers->points size after RANSAC: " << cloudOutliers->points.size() << std::endl;
    //
    // if(cloudInliers->points.size() == 0){
    //   std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    // }

    if(pcl_inliers->indices.size() == 0){
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult (cloudInliers, cloudOutliers);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(pcl_inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> pcl_clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction

    std::vector<std::vector<float>> points;
    for(size_t it=0; it<cloud->points.size(); it++)
    {
      // std::vector<float> point {cloud->points[it].x, cloud->points[it].y, cloud->points[it].z};
      points.push_back( std::vector<float> {cloud->points[it].x, cloud->points[it].y, cloud->points[it].z} );
    }

    KdTree* tree = new KdTree;
    for (int i=0; i<points.size(); i++)
    {
      // std::vector<float> point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    	tree->insert(points[i],i);
    }

    std::vector<pcl::PointIndices> cluster_indices;


    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);

    // int j = 0;
    for(int it = 0; it < clusters.size(); it++)
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new typename pcl::PointCloud<PointT>);

      for(int it2 = 0; it2 < clusters[it].size(); it2++)
        cloud_cluster->points.push_back (cloud->points[clusters[it][it2]]); //*

      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      if(cloud_cluster->points.size()>=minSize && cloud_cluster->points.size() <= maxSize){
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        pcl_clusters.push_back(cloud_cluster);
      }

      // std::stringstream ss;
      // ss << "cloud_cluster_" << j << ".pcd";
      // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
      // j++;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << pcl_clusters.size() << " clusters" << std::endl;

    return pcl_clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
