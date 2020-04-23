
#ifndef REGISTER_KINECT_THREAD_H
#define REGISTER_KINECT_THREAD_H

#include <QObject>
#include <QThread>
#include <QDateTime>
#include <QVariant>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <k4a/k4a.hpp>
#include <k4a/k4a.h>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <omp.h>
#include "Pixel.h"
#include "surfacematch.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include <halconcpp/HalconCpp.h>
#include <hdevengine/HDevEngineCpp.h>


using namespace HalconCpp;
class kinect_thread : public QObject {
Q_OBJECT
public:
    explicit kinect_thread(QObject *parent = nullptr);

    ~kinect_thread();
    void changeStatus(bool isStart);

signals:

    void sendFrame(int type,cv::Mat &mat);

    void sendDepth(QVariant modelVar);

public slots:

    void captureFrame();

    void captureDepth();

    void savePic();

    void savePointCloud();

private:
    uint32_t deviceCount;
    k4a::calibration calibration{};
    k4a_device_configuration_t config{};
    k4a::device device;
    std::vector<Pixel> depthTextureBuffer;
    uint8_t *irTextureBuffer{};
    uint8_t *colorTextureBuffer{};

    k4a::capture capture;
    k4a::image depthImage;
    k4a::image colorImage;
    k4a::image irImage,xy_table,point_cloud;
    cv::Mat depthFrame,colorFrame,irFrame;
    std::vector<double> point_x,point_y,point_z;
    HTuple tmpx,tmpy,tmpz,model3D;
    int point_count = 0;
    QVariant modelVar;


    bool isStartCapture, isConected;
    uint8_t picNumber = 0;//记录储存照片张数
    const char *colorImg = "./colorimg";
    const char *depthImg = "./depthimg";
    const char *irImg = "./irimg";
    const char *extension = ".png";
    void create_xy_table();
    void generate_point_cloud();
    void generate_point_cloud(const k4a::image depth_image);
    void write_point_cloud(const char *file_name);
};

#endif // KINECT_THREAD_H
