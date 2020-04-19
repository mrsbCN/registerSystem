#ifndef KINECT_DK_H
#define KINECT_DK_H

#include <iostream>
#include <QObject>
#include <QThread>
#include <QDateTime>
#include <k4a/k4a.h>
#include <Open3D/Open3D.h>

class kinect_dk : public QObject {
Q_OBJECT
public:
    explicit kinect_dk(QObject *parent = nullptr);

    void changeStatus(bool isStart);

    ~kinect_dk();
    open3d::io::AzureKinectSensorConfig sensor_config;  //传感器配置
    open3d::camera::PinholeCameraIntrinsic intrinsic;
private:
    int sensor_index;
    bool enable_align_depth_to_color;
    open3d::io::AzureKinectSensor *sensor;
    bool isStartCapture, isConected;
    std::shared_ptr<open3d::geometry::RGBDImage> rgbdImage;
    std::shared_ptr<open3d::geometry::PointCloud> pcd;

signals:

    void finished(int type);

    void sendFrame(const open3d::geometry::RGBDImage &rgbd);

    void sendDepth(const QString &pointCloudFile);

public slots: //槽函数
    void captureFrame();

    void captureDepth();

    void savePic();

    void savePointCloud();

};

#endif // KINECT_DK_H
