#include "mainwindow.h"

#include <iostream>
#include <QApplication>

Q_DECLARE_METATYPE(cv::Mat);
Q_DECLARE_METATYPE(k4a_capture_t);
Q_DECLARE_METATYPE(QVariant);
int main(int argc, char **argv) {

    qRegisterMetaType<cv::Mat>("cv::Mat");
    qRegisterMetaType<cv::Mat>("cv::Mat&");
    qRegisterMetaType<k4a_capture_t>("k4a_capture_t");
    qRegisterMetaType<k4a_capture_t>("k4a_capture_t&");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
