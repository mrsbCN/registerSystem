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
#include <Open3D/Open3D.h>
#include <Open3D/Registration/FastGlobalRegistration.h>
#include "mesh2pcd.h"
#include "kinect_dk.h"
#include "kinect_thread.h"
#include "ppfmatch.h"
#include "surfacematch.h"

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
    bool isTabCameraStart, isTabContinueStart;
    QString CADFileName, modelFileName, sceneFileName;
    QThread m_kinectThread,m_surfaceMatchThread;
    kinect_thread *m_kinect;
    PPFmatch *matcher;
    surfaceMatch *sfm;
    HTuple singleWindowHandle, continueWindowHandle;
    HTuple hv_ObjectModel3D, hv_ObjectScene3D, hv_ObjectModel3DRigidTrans;
    HTuple hv_Status, hv_PoseOut, matchPose,CamPose;
    HTuple *hv_ObjectScenePtr;
    QFuture<void> f;
    double matchScore;
    QImage Img;
    cv::Mat Rgb;

    void renderModel(const QString &fileName, HTuple &WindowHandle);

    void renderScene(bool isClickable,HTuple &WindowHandle);

protected slots://槽函数
    void onStartCameraButtonClicked();

    void onTabClicked(int current);

    void onSelectCADFileButtonClicked();

    void onSelectPointCloudFileButtonClicked();

    void onSelectSceneButtonClicked();

    void onBeginButtonClicked();

    void onSavePointCloudClicked();

    void onSavePicClicked();

    void TabCameraDisplay(int type,cv::Mat mat);

    void continueRead(QVariant modelVar);

    void onRegisterComplete(QVariant Pose, double Score);

    void test();

signals://信号
    void startRunning();

    void startSavePointCloud();

    void startSavePic();

    void startRead();

    void startMatch(double RelSamplingDistance, double KeyPointFraction);

};

#endif // MAINWINDOW_H
