#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    ui->lineEdit->setValidator(new QIntValidator(50, 300, this));
    ui->lineEdit_2->setValidator(new QIntValidator(1, 4, this));

    matcher = new PPFmatch();
    isTabCameraStart = false;
    std::cout << "I'm working in thread:" << QThread::currentThreadId() << std::endl;
    m_kinectDK = new kinect_dk();
    m_kinectDK->moveToThread(&m_kinectThread);

    connect(ui->startButton, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
    connect(ui->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(onTabClicked(int)));
    connect(ui->selectCADFileButton, SIGNAL(clicked()), this, SLOT(onSelectCADFileButtonClicked()));
    connect(ui->selectPointCloudFileButton, SIGNAL(clicked()), this, SLOT(onSelectPointCloudFileButtonClicked()));
    connect(ui->selectPointCloudFileButton_2, SIGNAL(clicked()), this, SLOT(onSelectSceneButtonClicked()));

    connect(this, SIGNAL(startRunning()), m_kinectDK, SLOT(captureFrame()));
    connect(m_kinectDK, SIGNAL(sendFrame(const open3d::geometry::RGBDImage &)),
            this, SLOT(TabCameraDisplay(const open3d::geometry::RGBDImage &)));

    m_kinectThread.start();
}

MainWindow::~MainWindow() {
    delete ui;
    m_kinectDK->changeStatus(false);
    m_kinectThread.quit();
    m_kinectThread.wait();
    delete m_kinectDK;
}

void MainWindow::onStartButtonClicked() {
    if (!isTabCameraStart)//未开始状态,转为开始
    {
        ui->startButton->setText("Stop");
        m_kinectDK->changeStatus(true);
        emit startRunning();
    } else {
        ui->startButton->setText("Start");
        m_kinectDK->changeStatus(false);
    }
    isTabCameraStart = !isTabCameraStart;
}

void MainWindow::onTabClicked(int current) {
    if (current == 0) {

    } else if (current == 1) {
        isTabCameraStart = false;
        ui->startButton->setText("Start");
        m_kinectDK->changeStatus(false);
    } else if (current == 2) {
        isTabCameraStart = false;
        ui->startButton->setText("Start");
        m_kinectDK->changeStatus(false);
    }
}

void myvisualization(const std::vector<std::shared_ptr<const open3d::geometry::Geometry >> &geometry_ptrs) {
    open3d::visualization::DrawGeometries(geometry_ptrs, "vis");
}

void MainWindow::onSelectCADFileButtonClicked() {
    fileName = QFileDialog::getOpenFileName(
            this,
            tr("open a CAD file."),
            "./",
            tr("mesh files(*.stl *.ply *.obj);;All files(*.*)"));//tr("images(*.png *jpeg *bmp);;video files(*.avi *.mp4 *.wmv);;All files(*.*)"));
//    std::cout<<fileName.toStdString()<<std::endl;
    if (!fileName.isEmpty()) {
//        std::vector< std::shared_ptr< const open3d::geometry::Geometry >>  geometry_ptrs;
//        std::allocator<open3d::geometry::TriangleMesh> alloc;
//        auto mesh_ptr = std::allocate_shared<open3d::geometry::TriangleMesh>(alloc);
//        open3d::io::ReadTriangleMesh(fileName.toStdString(), *mesh_ptr);
//        mesh_ptr->PaintUniformColor(Eigen::Vector3d(1,0,0));
//        geometry_ptrs.push_back(mesh_ptr);
//        QtConcurrent::run(myvisualization,geometry_ptrs);
        int resolu = ui->lineEdit->text().toInt();
        int tesselated = ui->lineEdit_2->text().toInt();
        QFuture<void> f = QtConcurrent::run(mystart, fileName, resolu, tesselated);
//        std::cout<<"finished"<<std::endl;
    }
}

void MainWindow::onSelectPointCloudFileButtonClicked() {
    fileName = QFileDialog::getOpenFileName(
            this,
            tr("open a PointCloud file."),
            "./",
            tr("mesh files(*.pcd *.ply);;All files(*.*)"));
    std::cout << fileName.toStdString() << std::endl;
    if (!fileName.isEmpty()) {
        double samplingStep = ui->lineEdit_3->text().toDouble();
        double distanceStep = ui->lineEdit_4->text().toDouble();
        double numAngles = ui->lineEdit_5->text().toDouble();
        matcher->setParameter(samplingStep,distanceStep,numAngles);
        matcher->train(fileName);

        //std::allocator<open3d::geometry::TriangleMesh> alloc;
        //auto point_ptr = std::allocate_shared<open3d::geometry::PointCloud>(alloc);
        //open3d::io::ReadPointCloud(fileName.toStdString(), *point_ptr);
        //std::vector< std::shared_ptr< const open3d::geometry::Geometry >>  geometry_ptrs;
        //geometry_ptrs.push_back(point_ptr);
        //QtConcurrent::run(myvisualization,geometry_ptrs);
    }
}

void MainWindow::onSelectSceneButtonClicked() {
    fileName = QFileDialog::getOpenFileName(
            this,
            tr("open a scene file."),
            "./",
            tr("mesh files(*.pcd *.ply);;All files(*.*)"));
    std::cout << fileName.toStdString() << std::endl;
    if (!fileName.isEmpty()) {
        double distanceStep = ui->lineEdit_6->text().toDouble();
        double scale = ui->lineEdit_7->text().toDouble();
        matcher->loadScenePointCloudFile(fileName);
        matcher->match(fileName);

    }
}

void MainWindow::TabCameraDisplay(const open3d::geometry::RGBDImage &rgbd) {
    colorImg = QImage(rgbd.color_.data_.data(), rgbd.color_.width_, rgbd.color_.height_,
                      rgbd.color_.width_ * rgbd.color_.num_of_channels_, QImage::Format_RGB888);
    colorImg = colorImg.scaled(ui->clolorframe->width(), ui->clolorframe->height());
    depthImg = QImage(rgbd.depth_.data_.data(), rgbd.depth_.width_, rgbd.depth_.height_,
                      rgbd.depth_.width_ * rgbd.depth_.num_of_channels_, QImage::Format_Grayscale8);
    depthImg = depthImg.scaled(ui->depthFrame->width(), ui->depthFrame->height());

    ui->clolorframe->setPixmap(QPixmap::fromImage(colorImg));
    ui->depthFrame->setPixmap(QPixmap::fromImage(depthImg));
    //std::cout<<ui->tabWidget->currentIndex()<<std::endl;
}
