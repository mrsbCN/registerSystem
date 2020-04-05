#include "kinect_dk.h"

kinect_dk::kinect_dk(QObject *parent) : QObject(parent) {
    sensor_config = open3d::io::AzureKinectSensorConfig();  //传感器配置
    sensor_index = 0;
    enable_align_depth_to_color = false;
    isStartCapture = false;
    if (!open3d::io::ReadIJsonConvertibleFromJSON("./config.json", sensor_config)) {
        open3d::utility::LogError("Invalid sensor config");
        //return 1;
    }
    //std::cout<<sensor_config.config_.at("depth_mode")<<std::endl;

    // Init sensor
    sensor = new open3d::io::AzureKinectSensor(sensor_config);
    if (!sensor->Connect(sensor_index)) {
        open3d::utility::LogError("Failed to connect to sensor, abort.");
        //return 1;
    }

    // Start viewing
    do {
        std::shared_ptr<open3d::geometry::RGBDImage> im_rgbd = sensor->CaptureFrame(enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            open3d::utility::LogInfo("Invalid capture, skipping this frame");
            continue;
        }
//        open3d::camera::PinholeCameraIntrinsic intrinsic = open3d::camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault;
//        open3d::geometry::Image depth = (im_rgbd->depth_);
//        auto pcd = open3d::geometry::PointCloud::CreateFromDepthImage(depth,intrinsic,Eigen::Matrix4d::Identity(),1000.0,1000.0,1,true);
//        pcd = pcd->VoxelDownSample(0.01f);
        break;
    } while (true);
}

kinect_dk::~kinect_dk() {
    sensor->~AzureKinectSensor();
}

void kinect_dk::changeStatus(bool isStart) {
    isStartCapture = isStart;
}

void kinect_dk::savePic() {

}

void kinect_dk::savePointCloud() {

}

void kinect_dk::captureFrame() {
    while (isStartCapture) {
        std::shared_ptr<open3d::geometry::RGBDImage> im_rgbd = sensor->CaptureFrame(enable_align_depth_to_color);
        if (im_rgbd == nullptr) {
            open3d::utility::LogInfo("Invalid capture, skipping this frame");
            continue;
            //QThread::msleep(10);
        }
        emit sendFrame(*im_rgbd);
    }
}
