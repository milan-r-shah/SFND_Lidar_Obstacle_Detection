/* \author: Aaron Brown & Milan R. Shah */

// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>

#include "../../processPointClouds.h"
#include "../../render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    cloud->width = cloud->points.size();
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
    // mesaure the time Ransac takes to fit a line
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    std::unordered_set<int> tempInliers;

    // For max iterations
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // Randomly pick 2 points to fit a line
        // Here, tempInliers is an unordered_set. So, in case, if rand() generates the second point 
        // same as previous one then it won't be inserted in the set.
        while (tempInliers.size() < 2)
            tempInliers.insert(rand() % cloud->size());

        auto itr = tempInliers.begin();
        pcl::PointXYZ pt1(cloud->points.at(*itr));
        itr++;
        pcl::PointXYZ pt2(cloud->points.at(*itr));

        // for Point1(x1, y1) & Point2(x2, y2), line equation is:
        // (y1 - y2) * x + (x2 - x1) * y + (x1 * y2 - x2 * y1) = 0;
        float A = pt1.y - pt2.y;
        float B = pt2.x - pt1.x;
        float C = pt1.x * pt2.y - pt2.x * pt1.y;

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for (int index = 0; index < cloud->size(); ++index) {
            if (tempInliers.count(index))
                continue;

            // Distance of a CurrentPoint(x0, y0) from the fitted line: Ax + By + C = 0:
            // dist = | A * x0 + B * y0 + C | / sqrt(A ^ 2 + B ^ 2);
            pcl::PointXYZ currPt = cloud->points.at(index);
            float dist = std::fabs(A * currPt.x + B * currPt.y + C) / std::sqrt(A * A + B * B);

            if (dist <= distanceTol)
                tempInliers.insert(index);
        }

        if (tempInliers.size() > inliersResult.size())
            inliersResult = tempInliers;

        tempInliers.clear();
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " milliseconds\n";

    // Return indicies of inliers from fitted line with most inliers
    return inliersResult;
}

int main() {
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
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