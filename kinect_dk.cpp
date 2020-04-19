#include "kinect_dk.h"

kinect_dk::kinect_dk(QObject *parent) : QObject(parent) {
    sensor_config = open3d::io::AzureKinectSensorConfig();  //传感器配置
    sensor_index = 0;
    enable_align_depth_to_color = false;
    isStartCapture = false;
    isConected = false;
    rgbdImage = nullptr;
    if (!open3d::io::ReadIJsonConvertibleFromJSON("./config.json", sensor_config)) {
        open3d::utility::LogError("Invalid sensor config");
        //return 1;
    }
    //intrinsic = open3d::camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault;
    intrinsic.SetIntrinsics(512,512,257.652,257.763,260.75,264.25);
    std::cout<<intrinsic.width_<<" "<<intrinsic.height_<<" "<<intrinsic.intrinsic_matrix_<<" "<<std::endl;

    // Init sensor
    sensor = new open3d::io::AzureKinectSensor(sensor_config);
    if (sensor->Connect(sensor_index)) {
        isConected = true;
        // Start viewing
        do {
            std::shared_ptr<open3d::geometry::RGBDImage> im_rgbd = sensor->CaptureFrame(enable_align_depth_to_color);
            if (im_rgbd == nullptr) {
                open3d::utility::LogInfo("Invalid capture, skipping this frame");
                continue;
            }
            std::cout<<im_rgbd->depth_.width_<<" "<<im_rgbd->depth_.height_<<std::endl;
            break;
        } while (true);
    } /*else{
        open3d::utility::LogError("Failed to connect to sensor, abort.");
    }*/
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
    while (isStartCapture && isConected) {
        std::shared_ptr<open3d::geometry::RGBDImage> rgbdImage = sensor->CaptureFrame(enable_align_depth_to_color);
        if (rgbdImage == nullptr) {
            open3d::utility::LogInfo("Invalid capture, skipping this frame");
            continue;
            //QThread::msleep(10);
        }
        emit sendFrame(*rgbdImage);
    }
}

void kinect_dk::captureDepth() {
    //qint64 begin = QDateTime::currentMSecsSinceEpoch();
    if (isStartCapture && isConected) {
        do{
            rgbdImage = sensor->CaptureFrame(enable_align_depth_to_color);
        }while (rgbdImage == nullptr);
        pcd = open3d::geometry::PointCloud::CreateFromDepthImage(rgbdImage->depth_, intrinsic, Eigen::Matrix4d::Identity(),
                                                                 1.0, 1000.0, 1, true);
        //pcd = pcd->VoxelDownSample(0.01f);
        qint64 time = QDateTime::currentMSecsSinceEpoch();
        QString fileName= "/dev/shm/"+tr("%1").arg(time)+".ply";
        open3d::io::WritePointCloudToPLY(fileName.toStdString(),*pcd);
        emit sendDepth(fileName);
    }
    //std::cout<<QDateTime::currentMSecsSinceEpoch() - begin<<std::endl;
}