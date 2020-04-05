#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QObject>
#include <QThread>
#include <QLabel>
#include <QImage>
#include <QFileDialog>
#include <QThread>
#include <QFuture>
#include <QtConcurrent>
#include <QIntValidator>
#include <memory>
#include "mesh2pcd.h"
//#include "generatepoincloud.h"
#include "kinect_dk.h"
#include "ppfmatch.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow();

private:
    Ui::MainWindow *ui;
    bool isTabCameraStart;
    QString fileName;
    QImage colorImg, depthImg;
    QThread m_kinectThread;
    kinect_dk *m_kinectDK;
    PPFmatch *matcher;

protected slots://槽函数
    void onStartButtonClicked();

    void onTabClicked(int current);

    void onSelectCADFileButtonClicked();

    void onSelectPointCloudFileButtonClicked();

    void onSelectSceneButtonClicked();

    void TabCameraDisplay(const open3d::geometry::RGBDImage &rgbd);

signals://信号
    void startRunning();
};

#endif // MAINWINDOW_H
