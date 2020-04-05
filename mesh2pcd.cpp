#include "mesh2pcd.h"


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

QString generateFileName(QString &inputFile) {
    int left = inputFile.lastIndexOf("/");
    int right = inputFile.lastIndexOf(".");
    QString outFile = "." + inputFile.mid(left, right - left) + ".pcd";
    std::cout << outFile.toStdString() << std::endl;
    return outFile;
}


int mystart(QString &fileName, int resolu, int tesselated) {
    // Parse command line arguments
    QString outputFile = generateFileName(fileName);
    int tesselated_sphere_level = tesselated;
    int resolution = resolu;
    vtkSmartPointer<vtkPolyData> polydata1;

    if (fileName.indexOf(".ply") != -1) {
        vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
        readerQuery->SetFileName(fileName.toStdString().data());
        readerQuery->Update();
        polydata1 = readerQuery->GetOutput();
    } else if (fileName.indexOf(".stl") != -1) {
        vtkSmartPointer<vtkSTLReader> readerQuery = vtkSmartPointer<vtkSTLReader>::New();
        readerQuery->SetFileName(fileName.toStdString().data());
        readerQuery->Update();
        polydata1 = readerQuery->GetOutput();
    } else if (fileName.indexOf(".obj") != -1) {
        vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
        readerQuery->SetFileName(fileName.toStdString().data());
        readerQuery->Update();
        polydata1 = readerQuery->GetOutput();
    } else {
        std::cout << "wrong file" << std::endl;
        return -1;
    }
    visualization::PCLVisualizer vis;
    vis.addModelFromPolyData(polydata1, "mesh1", 0);
    vis.setRepresentationToSurfaceForAllActors();

    PointCloud<PointXYZ>::CloudVectorType views_xyz;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<float> enthropies;
    std::cout << "---------" << std::endl;
    vis.renderViewTesselatedSphere(resolution, resolution, views_xyz, poses, enthropies, tesselated_sphere_level);
    std::cout << "---------" << std::endl;
    //take views and fuse them together
    std::vector<PointCloud<PointXYZ>::Ptr> aligned_clouds;

    for (std::size_t i = 0; i < views_xyz.size(); i++) {
        PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
        Eigen::Matrix4f pose_inverse;
        pose_inverse = poses[i].inverse();
        transformPointCloud(views_xyz[i], *cloud, pose_inverse);
        aligned_clouds.push_back(cloud);
    }

    // Fuse clouds
    PointCloud<PointXYZ>::Ptr big_boy(new PointCloud<PointXYZ>());
    for (const auto &aligned_cloud : aligned_clouds)
        *big_boy += *aligned_cloud;
    vis.addPointCloud(big_boy);
    //vis.spin();
    savePCDFileASCII(outputFile.toStdString(), *big_boy);
    return 0;
}
