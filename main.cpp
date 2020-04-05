#include "mainwindow.h"

#include <iostream>
#include <QApplication>

Q_DECLARE_METATYPE(open3d::geometry::RGBDImage);

int main(int argc, char **argv) {
    qRegisterMetaType<open3d::geometry::RGBDImage>("open3d::geometry::RGBDImage");
    qRegisterMetaType<open3d::geometry::RGBDImage>("open3d::geometry::RGBDImage&");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
