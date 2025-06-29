/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double        rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double        ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double        rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double        ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    cloud->width  = cloud->points.size();
    cloud->height = 1;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    // Time segmantation process
    auto start_time = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    // For max iterations

    // Randomly sample subset and fit line

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers

    // // v1.0
    // int num_points = cloud->points.size();

    // for (int i = 0; i < maxIterations; i++) {
    //     int idx1 = rand() % num_points;
    //     int idx2 = rand() % num_points;
    //     while (idx2 == idx1) {
    //         idx2 = rand() % num_points;
    //     }

    //     pcl::PointXYZ point1 = cloud->points[idx1];
    //     pcl::PointXYZ point2 = cloud->points[idx2];

    //     // Calculate the coefficients of the line (A, B, C) in the form Ax + By + C = 0
    //     float A = point1.y - point2.y;
    //     float B = point2.x - point1.x;
    //     float C = point1.x * point2.y - point2.x * point1.y;

    //     std::unordered_set<int> curr_inliers;

    //     for (int j = 0; j < num_points; j++) {
    //         if (j == idx1 || j == idx2) {
    //             continue; // Skip the points used to fit the line
    //         }

    //         pcl::PointXYZ curr_points = cloud->points[j];

    //         // Calculate the distance from the point to the line
    //         float distance = fabs(A * curr_points.x + B * curr_points.y + C) / std::sqrt(A * A + B * B);

    //         if (distance <= distanceTol) {
    //             curr_inliers.insert(j);  // If the distance is within the tolerance, add to inliers
    //         }
    //     }

    //     // If the current set of inliers is larger than the previous best, update the result
    //     if (curr_inliers.size() > inliersResult.size()) {
    //         inliersResult = curr_inliers;
    //     }
    // }

    // // v2.0
    while (maxIterations--) {
        std::unordered_set<int> inliers_idx;

        // Randomly pick two points
        while (inliers_idx.size() < 2) {
            inliers_idx.insert(rand() % cloud->points.size());
        }

        float x1, y1, x2, y2;

        auto itr = inliers_idx.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // Calculate the coefficients of the line (A, B, C) in the form Ax + By + C = 0
        float a = y1 - y2;
        float b = x2 - x1;
        float c = x1 * y2 - x2 * y1;

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip the points that are already inliers. This is to avoid checking the same points again
            // This is not strictly necessary, but it can improve performance
            // since we don't need to check the points that are already inliers
            if (inliers_idx.count(index)) {
                continue;
            }

            pcl::PointXYZ curr_point = cloud->points[index];
            float x3 = curr_point.x;
            float y3 = curr_point.y;

            // Calculate the distance from the point to the line
            float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);

            if (d <= distanceTol) {
                inliers_idx.insert(index);
            }
        }

        if (inliers_idx.size() > inliersResult.size()) {
            inliersResult = inliers_idx;  // Update the result if the current set of inliers is larger
        }
    }

    auto end_time     = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "RANSAC took " << elapsed_time.count() << " milliseconds." << std::endl;

    return inliersResult;
}

int main() {
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index)) {
            cloudInliers->points.push_back(point);
        } else {
            cloudOutliers->points.push_back(point);
        }
    }

    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
