#ifndef PPFMATCH_H
#define PPFMATCH_H

#include <iostream>
#include <QObject>
#include <QString>
#include <Open3D/Open3D.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <QTime>
#include <QtCore>
#include <memory>
#include <omp.h>
#include "xxh3.h"
#include <hash_map>
#include <iostream>
#include <boost/smart_ptr.hpp>
using namespace std;
using namespace __gnu_cxx;
#define hashSeed 42

typedef struct THash {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4i id;
    int i, ppfInd;
    double angle;
} THash;

typedef struct Pose_3D
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double alpha;
    double residual;
    double angle;
    size_t modelIndex;
    size_t numVotes;
    Eigen::Matrix4d pose;
    Eigen::Vector3d t;
    Eigen::Vector4d q;

} Pose_3D;

typedef struct Cluster
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<Pose_3D,Eigen::aligned_allocator<Pose_3D>> poses;
    size_t accu_votes;
} Cluster;

//1 define the hash function
struct hashPPF {
    size_t operator()(const Eigen::Vector4i &f) const {

        return XXH64(&f, sizeof(Eigen::Vector4i), hashSeed);
    }
};

//2 define the equal function
struct equalPPF {
    bool operator()(const Eigen::Vector4i &f1, const Eigen::Vector4i &f2) const {
        return (f1 == f2);
    }
};

class PPFmatch : public QObject {
Q_OBJECT
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit PPFmatch(const double samplingStep = 0.04, const double distanceStep = 0.04, const int numAngles = 30,
                      QObject *parent = nullptr);

    ~PPFmatch();
    void setParameter(const double samplingStep, const double distanceStep, const int numAngles);
    void train(const QString &fileName);

    void match(const QString &fileName,std::vector<Pose_3D,Eigen::aligned_allocator<Pose_3D>>& results,const double relativeSceneSampleStep, const double relativeSceneDistance);
    void loadScenePointCloudFile(const QString &fileName);
    void hashtableWrite(QString hashFile);

protected:
    double angleStep, angleStepRadians, distanceStep;
    double samplingStepRelative, distanceStepRelative;

    Eigen::Vector4d samplingStep;
    int numAngle,numRefPoints;
    open3d::geometry::AxisAlignedBoundingBox modelBox;
    double modelDiameter;
    typedef hash_multimap<Eigen::Vector4i, THash, hashPPF, equalPPF> hashMapType;
    //hashMapType *hashMap;
    std::shared_ptr<hashMapType> hashMap;
    double positionThreshold, rotationThreshold;
    bool isTrained;
    int sceneSampleStep;
    QString inputFile;
    std::shared_ptr<open3d::geometry::PointCloud> modelPointPtr, scenePointPtr;

    void loadModelPointCloudFile(const QString &fileName);

    static void computePPFFeature(const Eigen::Vector3d &pt1, const Eigen::Vector3d &nor1, const Eigen::Vector3d &pt2,
                           const Eigen::Vector3d &nor2, Eigen::Vector4d &f);

    static double computeAlpha(const Eigen::Vector3d &pt1, const Eigen::Vector3d &nor1, const Eigen::Vector3d &pt2);

    static void computeTransformRT(const Eigen::Vector3d &p1, const Eigen::Vector3d &n1, Eigen::Matrix3d &R, Eigen::Vector3d &t);

    static void rtToPose(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, Eigen::Matrix4d& Pose);
    static void getUnitXRotation(double angle, Eigen::Matrix3d& Rx);
    static bool Pose_3DClusters(const Cluster& a, const Cluster& b);
    static bool pose_3DCompare(const Pose_3D& a, const Pose_3D& b);
    void clusterPoses(std::vector<Pose_3D,Eigen::aligned_allocator<Pose_3D>>& poseList, int numPoses, std::vector<Pose_3D,Eigen::aligned_allocator<Pose_3D>> &finalPoses);
    bool matchPose(const Pose_3D& sourcePose, const Pose_3D& targetPose);
signals:

};

#endif // PPFMATCH_H
