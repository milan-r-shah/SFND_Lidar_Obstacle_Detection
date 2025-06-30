/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool             renderScene = false;
    std::vector<Car> cars        = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0.0);
    // std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pt_cloud = lidar->scan();
    // renderRays(viewer, lidar->position, lidar_pt_cloud);
    // renderPointCloud(viewer, lidar_pt_cloud, "Lidar Point Cloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> point_processor;  // on stack
    // ProcessPointClouds<pcl::PointXYZ>* point_processor = new ProcessPointClouds<pcl::PointXYZ>();  // on heap

    //
    // Segment the point cloud into obstacles and plane
    //
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud = point_processor.SegmentPlane(lidar_pt_cloud, 100, 0.2);
    // renderPointCloud(viewer, segment_cloud.first, "obstacle_cloud", Color(1, 0, 0));  // red
    // renderPointCloud(viewer, segment_cloud.second, "plane_cloud", Color(0, 1, 0));  // green

    //
    // Cluster the obstacles
    //
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = point_processor.Clustering(segment_cloud.first, 1.0, 3, 30);
    int                                              cluster_id     = 0;
    std::vector<Color>                               colors         = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloud_clusters) {
        std::cout << "cluster size ";
        point_processor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle_cloud" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

        Box box = point_processor.BoundingBox(cluster);
        renderBox(viewer, box, cluster_id);

        cluster_id++;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS) {
        viewer->addCoordinateSystem(1.0);
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display city block ---------
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>*  point_processor_i = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud       = point_processor_i->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, input_cloud, "input_cloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud = point_processor_i->FilterCloud(input_cloud, 0.2, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 7, 1, 1));
    renderPointCloud(viewer, filtered_cloud, "filtered_cloud");
}   

int main(int argc, char** argv) {
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle                            setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}
