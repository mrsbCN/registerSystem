//
// Created by mrsb on 2020/4/20.
//

#include "kinect_thread.h"

kinect_thread::kinect_thread(QObject *parent) : QObject(parent) {
    isConected = false;
    isStartCapture = false;
    deviceCount = k4a::device::get_installed_count();
    if (deviceCount == 0) {
        std::cout << "No K4A devices found." << std::endl;
        return ;
    }
    device = k4a::device::open(K4A_DEVICE_DEFAULT);
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.synchronized_images_only = true;
    device.start_cameras(&config);
    isConected = true;

    point_count = 0;
    calibration = device.get_calibration(config.depth_mode, config.color_resolution);
    xy_table = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                  calibration.depth_camera_calibration.resolution_width,
                                  calibration.depth_camera_calibration.resolution_height,
                                  calibration.depth_camera_calibration.resolution_width * (int) sizeof(k4a_float2_t));
    create_xy_table();
    point_cloud = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                     calibration.depth_camera_calibration.resolution_width,
                                     calibration.depth_camera_calibration.resolution_height,
                                     calibration.depth_camera_calibration.resolution_width *
                                     (int) sizeof(k4a_float3_t));
}

kinect_thread::~kinect_thread() {
    device.close();
    xy_table.reset();
    point_cloud.reset();
}

void kinect_thread::changeStatus(bool isStart) {
    isStartCapture = isStart;
}

void kinect_thread::savePic() {
    if (isConected) {
        std::cout << "savePic clicked" << int(picNumber) << std::endl;
        if (device.get_capture(&capture)) {
            colorImage = capture.get_color_image();
            depthImage = capture.get_depth_image();
            //std::cout<<depthImage.get_stride_bytes()<<std::endl;
            irImage = capture.get_ir_image();
            ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed,
                               GetDepthModeRange(config.depth_mode),
                               &depthTextureBuffer);
            irTextureBuffer = irImage.get_buffer();
            colorTextureBuffer = colorImage.get_buffer();
            depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4,
                                 depthTextureBuffer.data());
            colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4,
                                 colorTextureBuffer);//BBBBBBBB GGGGGGGG RRRRRRRR AAAAAAAA
            depthFrameNoColorized = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16UC1,
                                            depthImage.get_buffer());
            irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16UC1, irTextureBuffer);
            cv::imwrite(colorImg + std::to_string(picNumber) + extension, colorFrame);
            cv::imwrite(depthImg + std::to_string(picNumber) + extension, depthFrame);
            cv::imwrite(depthImgNoColorized + std::to_string(picNumber) + extension, depthFrameNoColorized);
            cv::imwrite(irImg + std::to_string(picNumber) + extension, irFrame);
            picNumber++;
        }

    }
}

void kinect_thread::savePointCloud() {
    if (isConected) {
        std::cout << "savePointCloud clicked" << std::endl;
        // Get a capture
        if (device.get_capture(&capture)) {
            qint64 time = QDateTime::currentMSecsSinceEpoch();
            depthImage = capture.get_depth_image();
            generate_point_cloud(depthImage);
            QString fileName = "/home/mrsb/" + tr("%1").arg(time) + ".ply";
            write_point_cloud(fileName.toStdString().c_str());
            std::cout << QDateTime::currentMSecsSinceEpoch() - time << std::endl;
        }
    }
}

void kinect_thread::captureFrame() {
    while (isStartCapture && isConected) {
        if (device.get_capture(&capture)) {
            colorImage = capture.get_color_image();
            depthImage = capture.get_depth_image();
            irImage = capture.get_ir_image();
            ColorizeDepthImage(depthImage, DepthPixelColorizer::ColorizeBlueToRed,
                               GetDepthModeRange(config.depth_mode),
                               &depthTextureBuffer);
            irTextureBuffer = irImage.get_buffer();
            colorTextureBuffer = colorImage.get_buffer();
            depthFrame = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_8UC4,
                                 depthTextureBuffer.data());
            colorFrame = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4,
                                 colorTextureBuffer);//BBBBBBBB GGGGGGGG RRRRRRRR AAAAAAAA
            irFrame = cv::Mat(irImage.get_height_pixels(), irImage.get_width_pixels(), CV_16UC1, irTextureBuffer);
            //cv::imshow("kinect depth map master", depthFrame);
            //cv::imshow("kinect color frame master", colorFrame);
            //cv::imshow("kinect ir frame master", irFrame);
            emit sendFrame(1, colorFrame);
            emit sendFrame(2, depthFrame);
            emit sendFrame(3, irFrame);
        }
    }
}

void kinect_thread::captureDepth() {
    if (isStartCapture && isConected) {
        if (device.get_capture(&capture)) {
            // Get a depth image
            qint64 time = QDateTime::currentMSecsSinceEpoch();
            //QString fileName = "/home/mrsb/" + tr("%1").arg(time) + ".ply";
            depthImage = capture.get_depth_image();
            generate_point_cloud();
            modelVar.setValue<HTuple>(model3D);
            emit sendDepth(modelVar);
            model3D.Clear();
            modelVar.clear();
            point_x.clear();
            point_y.clear();
            point_z.clear();
            std::cout<<QDateTime::currentMSecsSinceEpoch() - time<<std::endl;
        }
    }
}

void kinect_thread::create_xy_table() {
    k4a_float2_t *table_data = (k4a_float2_t *) (void *) (xy_table).get_buffer();

    int width = calibration.depth_camera_calibration.resolution_width;
    int height = calibration.depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++) {
        p.xy.y = (float) y;
        for (int x = 0; x < width; x++, idx++) {
            p.xy.x = (float) x;

            k4a_calibration_2d_to_3d(
                    &calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid) {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            } else {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

void kinect_thread::generate_point_cloud() {
    int width = depthImage.get_width_pixels();
    int height = depthImage.get_height_pixels();

    uint16_t *depth_data = (uint16_t *) (void *) depthImage.get_buffer();
    k4a_float2_t *xy_table_data = (k4a_float2_t *) (void *) xy_table.get_buffer();

    point_count = 0;
    for (int i = 0; i < width * height; i++) {
        if (depth_data[i] >100 && depth_data[i] <1000 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y)) {
            point_x.push_back(xy_table_data[i].xy.x * (float) depth_data[i] * 0.001);
            point_y.push_back(xy_table_data[i].xy.y * (float) depth_data[i] * 0.001);
            point_z.push_back((float) depth_data[i] * 0.001);
            (point_count)++;
        }
    }
    tmpx = HTuple(point_x.data(),point_count);
    tmpy = HTuple(point_y.data(),point_count);
    tmpz = HTuple(point_z.data(),point_count);
    GenObjectModel3dFromPoints(tmpx,tmpy,tmpz,&model3D);
}

void kinect_thread::generate_point_cloud(const k4a::image depth_image) {
    int width = depth_image.get_width_pixels();
    int height = depth_image.get_height_pixels();

    uint16_t *depth_data = (uint16_t *) (void *) depth_image.get_buffer();
    k4a_float2_t *xy_table_data = (k4a_float2_t *) (void *) xy_table.get_buffer();
    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)point_cloud.get_buffer();

    point_count = 0;
    for (int i = 0; i < width * height; ++i)
    {
        if (depth_data[i] >100 && depth_data[i] <1000 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }

}

void kinect_thread::write_point_cloud(const char *file_name) {
    int width = point_cloud.get_width_pixels();
    int height = point_cloud.get_height_pixels();

    k4a_float3_t *point_cloud_data = (k4a_float3_t *) (void *) point_cloud.get_buffer();

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << point_count << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (int i = 0; i < width * height; i++) {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z)) {
            continue;
        }

        ss << (float) point_cloud_data[i].xyz.x << " " << (float) point_cloud_data[i].xyz.y << " "
           << (float) point_cloud_data[i].xyz.z << std::endl;
    }

    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize) ss.str().length());
}