// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {
}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {
}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);

    // downsample the cloud using a voxel grid filter
    pcl::VoxelGrid<PointT> vxl_grd_filter;
    vxl_grd_filter.setInputCloud(cloud);
    vxl_grd_filter.setLeafSize(filterRes, filterRes, filterRes);
    vxl_grd_filter.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    // cropbox filter to remove points outside the specified region
    pcl::CropBox<PointT> cropbox_filter(true);
    cropbox_filter.setInputCloud(cloud_filtered);
    cropbox_filter.setMin(minPoint);
    cropbox_filter.setMax(maxPoint);
    cropbox_filter.filter(*cloud_region);

    std::vector<int> roof_point_indices_vec;
    // remove points that are not in the region of interest
    pcl::CropBox<PointT> roof_points_filter(true);
    roof_points_filter.setInputCloud(cloud_region);
    roof_points_filter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof_points_filter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof_points_filter.filter(roof_point_indices_vec);

    pcl::PointIndices::Ptr roof_point_indices{new pcl::PointIndices};
    for (int index : roof_point_indices_vec) {
        roof_point_indices->indices.push_back(index);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(roof_point_indices);
    extract.setNegative(true);  // Extract the points not in the roof region
    extract.filter(*cloud_region);

    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices) {
        plane_cloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  // Extract the points not in the inliers
    extract.filter(*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr       inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr  coefficients{new pcl::ModelCoefficients};
    // pcl::PointIndices::Ptr      inliers      = std::make_shared<pcl::PointIndices>();
    // pcl::ModelCoefficients::Ptr coefficients = std::make_shared<pcl::ModelCoefficients>();

    seg.setOptimizeCoefficients(true);  // optimize coefficients for better accuracy
    seg.setModelType(pcl::SACMODEL_PLANE);  // Set the model type to plane
    seg.setMethodType(pcl::SAC_RANSAC);  // Set the method type to RANSAC
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_tol) {
    std::unordered_set<int> inliers_result;
    srand(time(NULL));

    while (max_iterations--) {
        std::unordered_set<int> inliers_idx;

        // Randomly pick three points
        while (inliers_idx.size() < 3) {
            inliers_idx.insert(rand() % cloud->points.size());
        }

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers_idx.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // Use point1 as a reference and define two vectors on the plane v1 and v2
        // Vector v1 travels from point1 to point2
        // Vector v2 travels from point1 to point3
        pcl::PointXYZ v1 = pcl::PointXYZ(x2 - x1, y2 - y1, z2 - z1);
        pcl::PointXYZ v2 = pcl::PointXYZ(x3 - x1, y3 - y1, z3 - z1);

        // Calculate the normal vector of the plane using the cross product of v1 and v2
        pcl::PointXYZ normal_vector;
        normal_vector.x = v1.y * v2.z - v1.z * v2.y;
        normal_vector.y = v1.z * v2.x - v1.x * v2.z;
        normal_vector.z = v1.x * v2.y - v1.y * v2.x;

        // Check for collinear points, which results in a zero-length normal vector.
        // This would lead to division by zero later.
        float mag = sqrt(normal_vector.x * normal_vector.x + normal_vector.y * normal_vector.y + normal_vector.z * normal_vector.z);
        if (mag < 1e-6) {
            // Points are collinear, this is a degenerate plane. Skip this iteration.
            continue;
        }

        float A, B, C, D;
        A = normal_vector.x;
        B = normal_vector.y;
        C = normal_vector.z;

        D = -(A * x1 + B * y1 + C * z1);

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the points that are already inliers. This is to avoid checking the same points again
            // This is not strictly necessary, but it can improve performance
            // since we don't need to check the points that are already inliers
            if (inliers_idx.count(index)) {
                continue;
            }

            PointT curr_point = cloud->points[index];
            float  x4         = curr_point.x;
            float  y4         = curr_point.y;
            float  z4         = curr_point.z;

            // Calculate the distance from the point to the plane
            // float d = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A * A + B * B + C * C);
            float d = fabs(A * x4 + B * y4 + C * z4 + D) / mag;

            if (d <= distance_tol) {
                inliers_idx.insert(index);
            }
        }

        if (inliers_idx.size() > inliers_result.size()) {
            inliers_result = inliers_idx;  // Update the result if the current set of inliers is larger
        }
    }

    return inliers_result;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneAlgoFromScratch(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold) {
    // Time segmentation process
    auto start_time = std::chrono::steady_clock::now();

    std::unordered_set<int> inliers_unord_set = RansacPlane(cloud, max_iterations, distance_threshold);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

    for (int index : inliers_unord_set) {
        inliers->indices.push_back(index);
    }

    auto end_time     = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "RANSAC took " << elapsed_time.count() << " milliseconds." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr kd_tree(new pcl::search::KdTree<PointT>());
    kd_tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices>          cluster_indices_vec;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);  // Set the cluster tolerance
    ec.setMinClusterSize(minSize);  // Set the minimum cluster size
    ec.setMaxClusterSize(maxSize);  // Set the maximum cluster size
    ec.setSearchMethod(kd_tree);  // Set the search method
    ec.setInputCloud(cloud);  // Set the input cloud
    ec.extract(cluster_indices_vec);  // Extract the clusters

    for (pcl::PointIndices curr_indices : cluster_indices_vec) {
        typename pcl::PointCloud<PointT>::Ptr curr_cloud_cluster (new pcl::PointCloud<PointT>);

        for (int index : curr_indices.indices) {
            curr_cloud_cluster->points.push_back(cloud->points[index]);
        }

        curr_cloud_cluster->width    = curr_cloud_cluster->points.size();
        curr_cloud_cluster->height   = 1;  // Set height to 1 for unorganized point cloud
        curr_cloud_cluster->is_dense = true;  // Set is_dense to true

        clusters.push_back(curr_cloud_cluster);
    }

    auto endTime     = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file) {
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
