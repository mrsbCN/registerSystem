#include "mainwindow.h"

#include <iostream>
#include <QApplication>

Q_DECLARE_METATYPE(open3d::geometry::RGBDImage);
Q_DECLARE_METATYPE(open3d::geometry::Image);

int main(int argc, char **argv) {

    qRegisterMetaType<open3d::geometry::RGBDImage>("open3d::geometry::RGBDImage");
    qRegisterMetaType<open3d::geometry::RGBDImage>("open3d::geometry::RGBDImage&");
    qRegisterMetaType<open3d::geometry::Image>("open3d::geometry::Image");
    qRegisterMetaType<open3d::geometry::Image>("open3d::geometry::Image&");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
