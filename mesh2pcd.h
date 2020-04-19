#ifndef MESH2PCD_H
#define MESH2PCD_H

#include <QObject>
#include <QString>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>

extern int mystart(QString &filename, int resolu, int tesselated);

#endif // MESH2PCD_H
