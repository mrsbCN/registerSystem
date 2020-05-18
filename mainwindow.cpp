#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    SetHcppInterfaceStringEncodingIsUtf8(false);
    qRegisterMetaType<QVariant>("QVariant");
    ui->setupUi(this);
    ui->label_hit->setVisible(false);
    ui->lineEdit->setValidator(new QIntValidator(50, 300, this));
    ui->lineEdit_2->setValidator(new QIntValidator(1, 4, this));
    QRegExp double_rx(R"(1|^0\.(?!0{2})\d{1,2}$)");  //0-1
    ui->lineEdit_7->setValidator(new QRegExpValidator(double_rx, this));

    Hlong MainWndID = (Hlong) this->ui->single_widge->winId();
    OpenWindow(0, 0, 800, 640, MainWndID, "", "", &singleWindowHandle);
    SetWindowAttr("background_color", "black");
    set_display_font(singleWindowHandle, 14, "mono", "true", "false");
    HDevWindowStack::Push(singleWindowHandle);
    MainWndID = (Hlong) this->ui->continue_widget->winId();
    OpenWindow(0, 0, 800, 640, MainWndID, "", "", &continueWindowHandle);
    HDevWindowStack::Push(continueWindowHandle);

    sfm = new surfaceMatch();

    matcher = new PPFmatch();
    server = new myWebsocketServer(8080,true);
    //m_kinectDK = new kinect_dk();
    m_kinect = new kinect_thread();
    isTabCameraStart = false;
    isTabContinueStart = false;
    matchScore = 0.0f;
    hv_ObjectScenePtr = &(sfm->hv_ObjectScene3D);
    CreatePose(0, 0, 5,0, 0, 0, "Rp+T", "gba", "point", &CamPose);
    std::cout << "I'm working in thread:" << QThread::currentThreadId() << std::endl;

    m_kinect->moveToThread(&m_kinectThread);
    sfm->moveToThread(&m_surfaceMatchThread);

    connect(ui->startCameraButton, SIGNAL(clicked()), this, SLOT(onStartCameraButtonClicked()));
    connect(ui->saveCloudButton,SIGNAL(clicked()),this,SLOT(onSavePointCloudClicked()));
    connect(this,SIGNAL(startSavePointCloud()),m_kinect,SLOT(savePointCloud()),Qt::BlockingQueuedConnection);
    connect(ui->savePicButton,SIGNAL(clicked()),this,SLOT(onSavePicClicked()));
    connect(this,SIGNAL(startSavePic()),m_kinect,SLOT(savePic()),Qt::BlockingQueuedConnection);
    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(onTabClicked(int)));
    connect(ui->selectCADFileButton, SIGNAL(clicked()), this, SLOT(onSelectCADFileButtonClicked()));
    connect(ui->selectPointCloudFileButton, SIGNAL(clicked()), this, SLOT(onSelectPointCloudFileButtonClicked()));
    connect(ui->selectPointCloudFileButton_2, SIGNAL(clicked()), this, SLOT(onSelectSceneButtonClicked()));
    connect(ui->beginButton, SIGNAL(clicked()), this, SLOT(onBeginButtonClicked()));

    connect(this, SIGNAL(startRead()), m_kinect, SLOT(captureDepth()));
    connect(this, SIGNAL(startRunning()), m_kinect, SLOT(captureFrame()));
    connect(m_kinect, SIGNAL(sendFrame(int ,cv::Mat)),
            this, SLOT(TabCameraDisplay(int ,cv::Mat)));
    connect(m_kinect, SIGNAL(sendDepth(QVariant)),
            this, SLOT(continueRead(QVariant)),Qt::BlockingQueuedConnection);

    connect(this, SIGNAL(startMatch(double, double)), sfm,
            SLOT(continueMatch(double, double)));

    connect(sfm,SIGNAL(registerComplete(QVariant, double )),this,SLOT(onRegisterComplete(QVariant, double )));

    m_kinectThread.start();
    m_surfaceMatchThread.start(QThread::HighPriority);
}

MainWindow::~MainWindow() {
    delete ui;
    ClearWindow(singleWindowHandle);
    ClearWindow(continueWindowHandle);
    CloseWindow(singleWindowHandle);
    CloseWindow(continueWindowHandle);
    m_kinect->changeStatus(false);
    m_kinectThread.quit();
    m_kinectThread.wait();
    sfm->changeStatus(false);
    m_surfaceMatchThread.quit();
    m_surfaceMatchThread.wait();
    delete m_kinect;
    delete matcher;
    delete sfm;
    delete server;
}

void MainWindow::onStartCameraButtonClicked() {
    if (!isTabCameraStart)//未开始状态,转为开始
    {
        ui->startCameraButton->setText("Stop");
        m_kinect->changeStatus(true);
        emit startRunning();
    } else {
        ui->startCameraButton->setText("Start");
        m_kinect->changeStatus(false);
    }
    isTabCameraStart = !isTabCameraStart;
}

void MainWindow::onTabClicked(int current) {
    if (current == 0) {
        isTabContinueStart = false;
        ui->startCameraButton->setText("Start");
        m_kinect->changeStatus(false);
        ui->beginButton->setText("开始");
        sfm->changeStatus(false);
    } else if (current == 1) {
        isTabCameraStart = false;
        ui->startCameraButton->setText("Start");
        m_kinect->changeStatus(false);
        ui->beginButton->setText("开始");
        sfm->changeStatus(false);
    } else if (current == 2) {
        isTabCameraStart = false;
        isTabContinueStart = false;
        ui->startCameraButton->setText("Start");
        m_kinect->changeStatus(false);
        ui->beginButton->setText("开始");
        sfm->changeStatus(false);
    }
}

void MainWindow::onSelectCADFileButtonClicked() {
    CADFileName = QFileDialog::getOpenFileName(
            this,
            tr("open a CAD file."),
            "./",
            tr("mesh files(*.stl *.ply *.obj);;All files(*.*)"));//tr("images(*.png *jpeg *bmp);;video files(*.avi *.mp4 *.wmv);;All files(*.*)"));
//    std::cout<<fileName.toStdString()<<std::endl;
    if (!CADFileName.isEmpty()) {
        int resolu = ui->lineEdit->text().toInt();
        int tesselated = ui->lineEdit_2->text().toInt();
        //QFuture<void> f = QtConcurrent::run(mystart, CADFileName, resolu, tesselated);

        ClearObjectModel3d(hv_ObjectModel3D);
        ReadObjectModel3d(CADFileName.toStdString().c_str(), "mm", HTuple(), HTuple(), &hv_ObjectModel3D,
                          &hv_Status);
        visualize_object_model_3d(true,singleWindowHandle, hv_ObjectModel3D, HTuple(), HTuple(), HTuple(), HTuple(),
                                  HTuple(),
                                  HTuple(), HTuple(),
                                  &hv_PoseOut);
    }
}

void MainWindow::onSelectPointCloudFileButtonClicked() {
    modelFileName = QFileDialog::getOpenFileName(
            this,
            tr("open a PointCloud file."),
            "./",
            tr("mesh files(*.stl *.STL *.pcd *.ply);;All files(*.*)"));
    std::cout << modelFileName.toStdString() << std::endl;
    if (!modelFileName.isEmpty()) {
        double samplingStep = ui->lineEdit_3->text().toDouble();
        double distanceStep = ui->lineEdit_4->text().toDouble();
        int numAngles = ui->lineEdit_5->text().toInt();
        //matcher->setParameter(samplingStep, distanceStep, numAngles);
        //matcher->train(modelFileName);
        ui->label_hit->setVisible(true);
        renderModel(modelFileName,singleWindowHandle);
        sfm->train(f,hv_ObjectModel3D,samplingStep);
    }
}

void MainWindow::onSelectSceneButtonClicked() {
    sceneFileName = QFileDialog::getOpenFileName(
            this,
            tr("open a scene file."),
            "./",
            tr("mesh files(*.pcd *.ply);;All files(*.*)"));
    std::cout << sceneFileName.toStdString() << std::endl;
    if (!sceneFileName.isEmpty()) {
/*        vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> result;
        double distanceStep = ui->lineEdit_6->text().toDouble();
        double scale = ui->lineEdit_7->text().toDouble();
        matcher->loadScenePointCloudFile(sceneFileName);
        matcher->match(sceneFileName, result, scale, distanceStep);
        if (!result.empty()) {
            auto modelPointPtr = std::make_shared<open3d::geometry::PointCloud>();
            auto scenePointPtr = std::make_shared<open3d::geometry::PointCloud>();
            //open3d::geometry::PointCloud modelPointPtr,scenePointPtr;
            std::vector<std::shared_ptr<const open3d::geometry::Geometry> > geometryPtr;
            open3d::io::ReadPointCloud(modelFileName.toStdString(), *modelPointPtr);
            open3d::io::ReadPointCloud(sceneFileName.toStdString(), *scenePointPtr);
            modelPointPtr->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
            scenePointPtr->PaintUniformColor(Eigen::Vector3d(0, 1, 0));
            geometryPtr.push_back(scenePointPtr);
            modelPointPtr->Transform(result[0].pose);
            geometryPtr.push_back(modelPointPtr);
            open3d::visualization::DrawGeometries(geometryPtr);
            for (auto i :result) {
                geometryPtr.pop_back();
                modelPointPtr->Transform(i.pose);
                geometryPtr.push_back(modelPointPtr);
                open3d::visualization::DrawGeometries(geometryPtr);
            }*/
        ui->label_hit->setVisible(false);
        double distanceStep = ui->lineEdit_6->text().toDouble();
        double scale = ui->lineEdit_7->text().toDouble();
        ReadObjectModel3d(sceneFileName.toStdString().c_str(), "mm", HTuple(), HTuple(), hv_ObjectScenePtr,
                          &hv_Status);
        matchScore = sfm->match(*hv_ObjectScenePtr,distanceStep,scale,matchPose);
        double * pp = matchPose.ToDArr();
        std::cout<<pp[0]<<" "<<pp[1]<<" "<<pp[2]<<" "<<std::endl;
        renderScene(true,singleWindowHandle);

        }
}

void MainWindow::onBeginButtonClicked(){
    if (!isTabContinueStart)//未开始状态,转为开始
    {
        ui->beginButton->setText("停止");
        m_kinect->changeStatus(true);
        sfm->changeStatus(true);
        emit startRead();
        double distanceStep = ui->lineEdit_6->text().toDouble();
        double scale = ui->lineEdit_7->text().toDouble();
        emit startMatch(distanceStep,scale);
    } else {
        ui->beginButton->setText("开始");
        m_kinect->changeStatus(false);
        sfm->changeStatus(false);
    }
    isTabContinueStart = !isTabContinueStart;
}

void MainWindow::onSavePointCloudClicked(){
    if (isTabCameraStart)//如果已经开始就停掉
    {
        m_kinect->changeStatus(false);
        emit startSavePointCloud();
        m_kinect->changeStatus(true);
        emit startRunning();
    }
    else{
        emit startSavePointCloud();
    }
}

void MainWindow::onSavePicClicked(){
    if (isTabCameraStart)//未开始状态,转为开始
    {
        m_kinect->changeStatus(false);
        emit startSavePic();
        m_kinect->changeStatus(true);
        emit startRunning();
    }
    else{
        emit startSavePic();
    }
}

void MainWindow::TabCameraDisplay(int type,cv::Mat mat) {
    if (type == 1)
    {
        cv::cvtColor(mat,Rgb,CV_BGRA2RGBA);
        cv::resize(Rgb,Rgb,cv::Size(480,320));
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGBA8888);
        ui->clolorframe->setPixmap(QPixmap::fromImage(Img));
    }
    else if(type == 2)
    {
        cv::cvtColor(mat,Rgb,CV_BGRA2RGBA);
        cv::resize(Rgb,Rgb,cv::Size(480,320));
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGBA8888);
        ui->depthFrame->setPixmap(QPixmap::fromImage(Img));
    }
    else if(type == 3)
    {
        Rgb = mat;
        cv::resize(Rgb,Rgb,cv::Size(480,320));
        Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels() * 2, QImage::Format_Grayscale16);
        ui->irFrame->setPixmap(QPixmap::fromImage(Img));
    }

}

void MainWindow::continueRead(QVariant modelVar) {
    //互斥量
    sfm->readMutex.lock();
    hv_ObjectScenePtr->Clear();
    *hv_ObjectScenePtr = modelVar.value<HTuple>();
/*  ReadObjectModel3d(pointCloudFile.toStdString().c_str(), "mm", HTuple(), HTuple(), hv_ObjectScenePtr,
                      &hv_Status);*/
    sfm->readMutex.unlock();
    if(isTabContinueStart)
            emit startRead();
/*  QFile fileTemp(pointCloudFile);
    fileTemp.remove();*/
    renderScene(false,continueWindowHandle);

}

void MainWindow::renderModel(const QString &fileName, HTuple &WindowHandle) {
    ReadObjectModel3d(fileName.toStdString().c_str(), "mm", HTuple(), HTuple(), &hv_ObjectModel3D,
                      &hv_Status);
    visualize_object_model_3d(true,WindowHandle, hv_ObjectModel3D, HTuple(), HTuple(), (HTuple("point_size").Append("disp_normals")), (HTuple(2).Append("true")), HTuple(),
                              HTuple(), HTuple(),
                              &hv_PoseOut);
}

void MainWindow::renderScene(bool isClickable,HTuple &WindowHandle) {

    if(matchScore < 0.1)
    {
        std::cout<<matchScore<<std::endl;
        HTuple message = "Not Found";
        visualize_object_model_3d(isClickable,WindowHandle, *hv_ObjectScenePtr,
                                  HTuple(), CamPose, "point_size_0",
                                  1.0, HTuple(), message,
                                  HTuple(), &hv_PoseOut);
/*        std::cout<<hv_PoseOut.ToDArr()[0]<<" "<<hv_PoseOut.ToDArr()[1]<<" "<<hv_PoseOut.ToDArr()[2]<<" "
                 <<hv_PoseOut.ToDArr()[3]<<" "<<hv_PoseOut.ToDArr()[4]<<" "<<hv_PoseOut.ToDArr()[5]<<" "<<std::endl;*/
        return;
    }
    RigidTransObjectModel3d(hv_ObjectModel3D, matchPose, &hv_ObjectModel3DRigidTrans);
    HTuple hv_NumResult = hv_ObjectModel3DRigidTrans.TupleLength();
    HTuple hv_Colors;
    TupleGenConst(hv_NumResult, "green", &hv_Colors);
    HTuple hv_Indices = HTuple::TupleGenSequence(1,hv_NumResult,1);
    HTuple hv_GemParamName = ("color_"+(HTuple(0).TupleConcat(hv_Indices))).TupleConcat("point_size_0");
    HTuple hv_GemParamValue = (HTuple("gray").TupleConcat(hv_Colors)).TupleConcat(1.0);
    HTuple hv_showObject = (*hv_ObjectScenePtr).TupleConcat(hv_ObjectModel3DRigidTrans);

    visualize_object_model_3d(isClickable,WindowHandle, hv_showObject,
                              HTuple(), CamPose, hv_GemParamName,
                              hv_GemParamValue, HTuple(), HTuple(),
                              HTuple(), &hv_PoseOut);
/*    std::cout<<hv_PoseOut.ToDArr()[0]<<" "<<hv_PoseOut.ToDArr()[1]<<" "<<hv_PoseOut.ToDArr()[2]<<" "
            <<hv_PoseOut.ToDArr()[3]<<" "<<hv_PoseOut.ToDArr()[4]<<" "<<hv_PoseOut.ToDArr()[5]<<" "<<std::endl;*/
}

void MainWindow::onRegisterComplete(QVariant Pose, double Score){
    matchScore = Score;
    matchPose = Pose.value<HTuple>();
    std::cout<<matchScore<<std::endl;

    if(matchScore > 0.1)
    {
        flatbuffers::FlatBufferBuilder FlatBuilder(1024);
        ExchangeSerialization::PoseDateBuilder PoseBuilder(FlatBuilder);
        auto pose = ExchangeSerialization::Vec7(matchPose[0],matchPose[1],matchPose[2],
                                                matchPose[3],matchPose[4],matchPose[5],matchPose[6]);
        PoseBuilder.add_Pose(&pose);
        PoseBuilder.add_Timestamp(QDateTime::currentMSecsSinceEpoch());
        auto orc = PoseBuilder.Finish();
        FlatBuilder.Finish(orc);
        //qDebug() << FlatBuilder.GetSize();
        QByteArray buf((char *) FlatBuilder.GetBufferPointer(), FlatBuilder.GetSize());
        server->sendBinaryMessage(buf);
    }
}

void MainWindow::test() {
    flatbuffers::FlatBufferBuilder FlatBuilder(1024);
    ExchangeSerialization::PoseDateBuilder PoseBuilder(FlatBuilder);
    auto pose = ExchangeSerialization::Vec7(1.1,2,3,4,5,6,7);
    PoseBuilder.add_Pose(&pose);
    PoseBuilder.add_Timestamp(123);
    auto orc = PoseBuilder.Finish();
    FlatBuilder.Finish(orc);
    QByteArray buf("s");
    server->sendBinaryMessage(buf);
}