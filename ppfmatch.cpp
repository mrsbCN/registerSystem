#include "ppfmatch.h"

#define EPS     1e-8


PPFmatch::PPFmatch(const double samplingStep, const double distanceStep, const double numAngles, QObject *parent)
        : QObject(parent) {
    samplingStepRelative = samplingStep;
    distanceStepRelative = distanceStep;
    sceneSampleStep = (int) (1 / 0.04);
    angleStepRelative = numAngles;
    angleStepRadians = (360.0 / angleStepRelative) * M_PI / 180.0;
    angleStep = angleStepRadians;
    isTrained = false;
    modelPointPtr = std::make_shared<open3d::geometry::PointCloud>();
    scenePointPtr = std::make_shared<open3d::geometry::PointCloud>();
    std::cout << "finished" << std::endl;
}

PPFmatch::~PPFmatch() {
}

void PPFmatch::setParameter(const double samplingStep, const double distanceStep, const double numAngles){
    samplingStepRelative = samplingStep;
    distanceStepRelative = distanceStep;
    sceneSampleStep = (int) (1 / 0.04);
    angleStepRelative = numAngles;
    angleStepRadians = (360.0 / angleStepRelative) * M_PI / 180.0;
    angleStep = angleStepRadians;
    isTrained = false;
}

void PPFmatch::loadModelPointCloudFile(const QString &fileName) {
    inputFile = fileName;
    open3d::io::ReadPointCloud(fileName.toStdString(), *modelPointPtr);
    modelBox = modelPointPtr->GetAxisAlignedBoundingBox();
    Eigen::Vector3d range = modelBox.GetMaxBound() - modelBox.GetMinBound();
    modelDiameter = sqrt((range.array().square()).sum());
    std::cout << modelDiameter << std::endl;
    std::cout << "point number before Sample:" << modelPointPtr->points_.size() << std::endl;
    distanceStep = distanceStepRelative * modelDiameter;
    modelPointPtr = modelPointPtr->VoxelDownSample(distanceStep); //单位mm
    modelPointPtr->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(30));
    modelPointPtr->OrientNormalsTowardsCameraLocation();
    std::cout << "point number after Sample:" << modelPointPtr->points_.size() << std::endl;

    //open3d::visualization::DrawGeometries({point_ptr},"PointCloud",640,480,50,50);
}

void PPFmatch::loadScenePointCloudFile (const QString &fileName) {

    if(isTrained){
        open3d::io::ReadPointCloud(fileName.toStdString(), *scenePointPtr);
        std::cout << "point number before Sample:" << scenePointPtr->points_.size() << std::endl;
        scenePointPtr = scenePointPtr->VoxelDownSample(distanceStep); //单位mm
        scenePointPtr->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(30));
        scenePointPtr->OrientNormalsTowardsCameraLocation();
        std::cout << "point number after Sample:" << scenePointPtr->points_.size() << std::endl;
    }
    else{
        std::cout<<"Please select model first."<<std::endl;
    }
}

void PPFmatch::computePPFFeature(const Eigen::Vector3d &pt1, const Eigen::Vector3d &nor1, const Eigen::Vector3d &pt2,
                                 const Eigen::Vector3d &nor2, Eigen::Vector4d &f) {
    //nor1,nor2是单位向量，余弦定理不用除
    //离线阶段提前算角（n1,n2) f[3]
    Eigen::Vector3d d = pt2 - pt1;
    double norm = d.norm();
    f[0] = norm;
    if (f[0] <= EPS)
        return;
    d.normalize();

    f[1] = acos(nor1.dot(d));
    f[2] = acos(nor2.dot(d));
    f[3] = acos(nor1.dot(nor2));
}

void
PPFmatch::computePPFFeatureWithout1(const Eigen::Vector3d &pt1, const Eigen::Vector3d &nor1, const Eigen::Vector3d &pt2,
                                    const Eigen::Vector3d &nor2, Eigen::Vector4d &f) {
    //nor1,nor2是单位向量，余弦定理不用除
    //离线阶段提前算距离f[0]
    Eigen::Vector3d d = pt2 - pt1;
    float norm = d.norm();
    f[0] = norm;
    if (f[0] < EPS) //两点太近
        return;
    d.normalize();

    f[1] = acos(nor1.dot(d));
    f[2] = acos(nor2.dot(d));
    f[3] = acos(nor1.dot(nor2));
}

void PPFmatch::computetransformRT(const Eigen::Vector3d &p1, const Eigen::Vector3d &n1, Eigen::Matrix3d &R,
                                  Eigen::Vector3d &t) {
    // dot product with x axis
    double angle = acos(n1[0]);

    // cross product with x axis
    Eigen::Vector3d axis(0, n1[2], -n1[1]);

    // we try to project on the ground plane but it's already parallel
    if (n1[1] == 0 && n1[2] == 0) {
        axis[1] = 1;
        axis[2] = 0;
    } else {
        axis.normalize();
    }

    Eigen::AngleAxisd v(angle, Eigen::Vector3d(0, 0, 1));
    R = v.toRotationMatrix();
    t = -R * p1;
}

double PPFmatch::computeAlpha(const Eigen::Vector3d &pt1, const Eigen::Vector3d &nor1, const Eigen::Vector3d &pt2) {
    Eigen::Vector3d Tmg, mpt;
    Eigen::Matrix3d R;
    computetransformRT(pt1, nor1, R, Tmg);
    mpt = Tmg + R * pt2;
    double alpha = atan2(-mpt[2], mpt[1]);

    if (alpha != alpha) {
        return 0;
    }

    if (sin(alpha) * mpt[2] > 0.0)
        alpha = -alpha;

    return (alpha);
}

void PPFmatch::train(const QString &fileName) {
    loadModelPointCloudFile(fileName);
    samplingStep = Eigen::Vector4d(distanceStep, angleStepRadians, angleStepRadians, angleStepRadians);
    numRefPoints = modelPointPtr->points_.size();

    int validPPF = 0;
    qint64 start = QDateTime::currentDateTime().toMSecsSinceEpoch();
    for (int i = 0; i < numRefPoints; ++i) {
        const Eigen::Vector3d refPoint = modelPointPtr->points_[i];
        const Eigen::Vector3d refNormal = modelPointPtr->normals_[i];
        for (int j = 0; j < numRefPoints; ++j)  //考虑改成for(int j=0;j<i,++j)
        {
            if (i != j) {
                const Eigen::Vector3d pairPoint = modelPointPtr->points_[j];
                const Eigen::Vector3d pairNormal = modelPointPtr->normals_[j];
                Eigen::Vector4d feature(0, 0, 0, 0);
                float seeAble = refNormal.dot(pairNormal);  //a.b=|a||b|cos(theta)
                if (seeAble > EPS)                           //>0,即cos(theta)>0,夹角为(-90,90)认为是相互可见的 视点可见性
                {
                    ++validPPF;
                }
            }
        }
    }

    hashMap = std::make_shared<hashMapType>(validPPF);
    for (int i = 0; i < numRefPoints; ++i) {
        const Eigen::Vector3d refPoint = modelPointPtr->points_[i];
        const Eigen::Vector3d refNormal = modelPointPtr->normals_[i];
//#if defined _OPENMP
//#pragma omp parallel for
//#endif
//#pragma omp atomic
        for (int j = 0; j < numRefPoints; ++j)  //考虑改成for(int j=0;j<i,++j)
        {
            if (i != j) {
                const Eigen::Vector3d pairPoint = modelPointPtr->points_[j];
                const Eigen::Vector3d pairNormal = modelPointPtr->normals_[j];

                float seeAble = refNormal.dot(pairNormal);  //a.b=|a||b|cos(theta)
                if (seeAble > EPS)                           //>0,即cos(theta)>0,夹角为(-90,90)认为是相互可见的 视点可见性
                {
                    Eigen::Vector4d feature(0, 0, 0, 0);
                    computePPFFeature(refPoint, refNormal, pairPoint, pairNormal, feature);
                    Eigen::Vector4i sampledFeature = (feature.array() / samplingStep.array()).cast<int>();
                    double alpha = computeAlpha(refPoint, refNormal, pairPoint);
                    THash node;
                    node.id = sampledFeature;
                    node.i = i;
                    node.ppfInd = i * numRefPoints + j;
                    node.angle = alpha;
                    hashMap->insert(std::make_pair(sampledFeature, node));
                }
            }
        }
    }
    std::cout << "time uesd:" << QDateTime::currentDateTime().toMSecsSinceEpoch() - start << "ms" << std::endl;
    isTrained = true;
}

void PPFmatch::match(const QString &fileName) {

}