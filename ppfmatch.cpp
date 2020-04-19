#include "ppfmatch.h"

#define EPS     1e-8


PPFmatch::PPFmatch(const double samplingStep, const double distanceStep, const int numAngles, QObject *parent)
        : QObject(parent) {
    samplingStepRelative = samplingStep;
    distanceStepRelative = distanceStep;
    this->distanceStep = distanceStep;
    numRefPoints = 0;
    modelDiameter = 0.1f;
    sceneSampleStep = (int) (1 / 0.04);
    numAngle = numAngles;//30
    angleStepRadians = (360.0 / numAngle) * M_PI / 180.0;
    angleStep = angleStepRadians;
    //TODO 存疑
    positionThreshold = samplingStepRelative;
    rotationThreshold = ((360 / angleStep) / 180.0 * M_PI);
    isTrained = false;
    modelPointPtr = std::make_shared<open3d::geometry::PointCloud>();
    scenePointPtr = std::make_shared<open3d::geometry::PointCloud>();
    std::cout << "finished" << std::endl;
}

PPFmatch::~PPFmatch() {
}

void PPFmatch::setParameter(const double sampleStep, const double distStep, const int numAngles) {
    samplingStepRelative = sampleStep;
    distanceStepRelative = distStep;
    sceneSampleStep = (int) (1 / 0.04);
    numAngle = numAngles;
    angleStepRadians = (360.0 / numAngle) * M_PI / 180.0;
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
    modelPointPtr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(15, 30));
    modelPointPtr->OrientNormalsTowardsCameraLocation();
    modelPointPtr = modelPointPtr->VoxelDownSample(distanceStep); //单位mm
    std::cout << "point number after Sample:" << modelPointPtr->points_.size() << std::endl;

    //open3d::visualization::DrawGeometries({modelPointPtr}, "PointCloud", 640, 480, 50, 50);
}

void PPFmatch::loadScenePointCloudFile(const QString &fileName) {
    //TODO去除离群点
    if (isTrained) {
        open3d::io::ReadPointCloud(fileName.toStdString(), *scenePointPtr);
        auto box = modelPointPtr->GetAxisAlignedBoundingBox();
        Eigen::Vector3d range = box.GetMaxBound() - box.GetMinBound();
        double diameter = sqrt((range.array().square()).sum());
        std::cout << "point number before Sample:" << scenePointPtr->points_.size() << std::endl;
        scenePointPtr->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(15, 30));
        scenePointPtr->OrientNormalsTowardsCameraLocation();
        scenePointPtr = scenePointPtr->VoxelDownSample(diameter * samplingStepRelative); //单位mm
        std::cout << "point number after Sample:" << scenePointPtr->points_.size() << std::endl;
    } else {
        std::cout << "Please select model first." << std::endl;
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

void PPFmatch::computeTransformRT(const Eigen::Vector3d &p1, const Eigen::Vector3d &n1, Eigen::Matrix3d &R,
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
    computeTransformRT(pt1, nor1, R, Tmg);
    mpt = Tmg + R * pt2;
    double alpha = atan2(-mpt[2], mpt[1]);

    if (alpha != alpha) {
        return 0;
    }

    if (sin(alpha) * mpt[2] < 0.0)
        alpha = -alpha;

    return (-alpha);
}

void PPFmatch::rtToPose(const Eigen::Matrix3d &R, const Eigen::Vector3d &t, Eigen::Matrix4d &Pose) {
    //Matx34d P;
    //hconcat(R, t, P);
    //vconcat(P, Matx14d(0, 0, 0, 1), Pose);
    Eigen::Matrix4d P;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P(i, j) = R(i, j);
        }
    }

    for (int i = 0; i < 3; i++) {
        P(i, 3) = t(i, 0);
    }

    P(3, 0) = 0;
    P(3, 1) = 0;
    P(3, 2) = 0;
    P(3, 3) = 1;

    Pose = P;
}

bool PPFmatch::Pose_3DClusters(const Cluster &a, const Cluster &b) {
    return (a.accu_votes > b.accu_votes);
}

bool PPFmatch::pose_3DCompare(const Pose_3D &a, const Pose_3D &b) {
    return (a.numVotes > b.numVotes);
}

bool PPFmatch::matchPose(const Pose_3D &sourcePose, const Pose_3D &targetPose) {
    // translational difference
    Eigen::Vector3d dv = targetPose.t - sourcePose.t;
    double dNorm = dv.norm();
    const double phi = fabs(sourcePose.angle - targetPose.angle);

    return (phi < this->rotationThreshold && dNorm < this->positionThreshold);
}

void PPFmatch::getUnitXRotation(double angle, Eigen::Matrix3d &Rx) {
    const double sx = sin(angle);
    const double cx = cos(angle);

    //Mat(Rx.eye()).copyTo(Rx);
    Rx(0, 0) = 1, Rx(1, 0) = 0, Rx(2, 0) = 0;
    Rx(0, 1) = 0, Rx(1, 1) = 1, Rx(2, 1) = 0;
    Rx(0, 2) = 0, Rx(1, 2) = 0, Rx(2, 2) = 1;

    Rx(1, 1) = cx;
    Rx(1, 2) = -sx;
    Rx(2, 1) = sx;
    Rx(2, 2) = cx;
}

void PPFmatch::clusterPoses(std::vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> &poseList, int numPoses,
                            std::vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> &finalPoses) {

    std::vector<Cluster, Eigen::aligned_allocator<Cluster>> poseClusters;
    finalPoses.clear();
    // sort the poses for stability
    std::sort(poseList.begin(), poseList.end(), pose_3DCompare);

    for (int i = 0; i < numPoses; i++) {
        Pose_3D pose = poseList[i];
        bool assigned = false;
        // search all clusters
        for (size_t j = 0; j < poseClusters.size() && !assigned; j++) {
            //const Pose_3D poseCenter = poseClusters[j]->poseList[0];
            const Pose_3D poseCenter = poseClusters[j].poses[0];
            if (matchPose(pose, poseCenter)) {
                //poseClusters[j]->addPose(pose);
                poseClusters[j].poses.push_back(pose);
                assigned = true;
                break;
            }
        }

        if (!assigned) {
            Cluster poseCluster;
            poseCluster.poses.push_back(pose);
            poseClusters.push_back(poseCluster);
        }
    }
    // sort the clusters so that we could output multiple hypothesis
    std::sort(poseClusters.begin(), poseClusters.end(), Pose_3DClusters);

    finalPoses.resize(poseClusters.size());

    // TODO: Use MinMatchScore

/*#if defined _OPENMP
#pragma omp parallel for
#endif*/
    for (int i = 0; i < static_cast<int>(poseClusters.size()); i++) {
        // We could only average the quaternions. So I will make use of them here
        Eigen::Vector4d qAvg(0, 0, 0, 0);
        Eigen::Vector3d tAvg(0, 0, 0);

        // Perform the final averaging
        Cluster curCluster = poseClusters[i];
        std::vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> curPoses = curCluster.poses;
        const int curSize = (int) curPoses.size();

        for (int j = 0; j < curSize; j++) {
            qAvg += curPoses[j].q;
            tAvg += curPoses[j].t;
        }

        tAvg *= 1.0 / curSize;
        qAvg *= 1.0 / curSize;

        //curPoses[0]->updatePoseQuat(qAvg, tAvg);
        curPoses[0].numVotes = curCluster.accu_votes;

        finalPoses[i].pose = curPoses[0].pose;
        finalPoses[i].q = curPoses[0].q;
        finalPoses[i].t = curPoses[0].t;
        finalPoses[i].angle = curPoses[0].angle;
    }

    poseClusters.clear();
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

void
PPFmatch::match(const QString &fileName, std::vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> &results,
                const double relativeSceneSampleStep,
                const double relativeSceneDistance) {
    //TODO 检测场景点云已经更新
    //relativeSceneSampleStep取整个场景点的比例
    //relativeSceneDistance是点云采样栅格大小 （暂时不用）
    qint64 start = QDateTime::currentDateTime().toMSecsSinceEpoch();
    sceneSampleStep = (int) (1.0 / relativeSceneSampleStep);
    uint numPosesAdded = 0;
    int sceneSize = modelPointPtr->points_.size();
    std::vector<Pose_3D, Eigen::aligned_allocator<Pose_3D>> poseList;
    poseList.reserve((int) (sceneSize / sceneSampleStep) + 4);//每个参考点一个位姿

    open3d::geometry::KDTreeFlann kDTreePoint;
    kDTreePoint.SetGeometry(*scenePointPtr);
    std::cout << "-----------";
#if defined _OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < sceneSize; i += sceneSampleStep) {

        const Eigen::Vector3d refPoint = modelPointPtr->points_[i];
        const Eigen::Vector3d refNormal = modelPointPtr->normals_[i];
        uint refIndMax = 0, alphaIndMax = 0;
        uint maxVotes = 1;

        Eigen::Vector3d tsg(0, 0, 0);
        Eigen::Matrix3d Rsg = Eigen::Matrix3d::Zero(), RInv = Eigen::Matrix3d::Zero();

        //TODO bug ----用new不会置0
        uint *accumulator = (uint *) calloc(numAngle * numRefPoints, sizeof(uint));
        boost::shared_array<uint> accumulatorPtr(accumulator);
/*        Eigen::ArrayXXi acc = Eigen::ArrayXXi::Zero(numAngle, numRefPoints);
        Eigen::ArrayXXi::Index maxRow, maxCol;
        int max = acc.maxCoeff(&maxRow,&maxCol);*/;
        computeTransformRT(refPoint, refNormal, Rsg, tsg);
        std::vector<int> pairPointIndex;
        std::vector<double> pairPointDistance;
        kDTreePoint.SearchRadius(refPoint, modelDiameter * 0.75, pairPointIndex, pairPointDistance);
        //TODO 第二项参数大小待定
        //for(int j=0;j<sceneSize;++j){
        for (int j :pairPointIndex) {
            const Eigen::Vector3d pairPoint = scenePointPtr->points_[j];
            const Eigen::Vector3d pairNormal = scenePointPtr->normals_[j];

            //Eigen::Vector3d p2t;
            double alpha_scene;

            Eigen::Vector4d feature(0, 0, 0, 0);
            computePPFFeature(refPoint, refNormal, pairPoint, pairNormal, feature);
            Eigen::Vector4i sampledFeature = (feature.array() / samplingStep.array()).cast<int>();

            alpha_scene = computeAlpha(refPoint, refNormal, pairPoint);
/*            p2t = tsg + Rsg * Eigen::Vector3d(pairPoint);
            alpha_scene = atan2(-p2t[2], p2t[1]);
            if (alpha_scene != alpha_scene) {
                continue;
            }
            if (sin(alpha_scene) * p2t[2] < 0.0)
                alpha_scene = -alpha_scene;
            alpha_scene = -alpha_scene;*/
            auto iter = hashMap->equal_range(sampledFeature);
            for_each(iter.first, iter.second, [=](hashMapType::value_type &x) {
                THash &tData = x.second;
                int corrI = (int) tData.i;
                int ppfInd = (int) tData.ppfInd;
                double alpha_model = tData.angle;
                double alpha = alpha_model - alpha_scene;
                int alpha_index = (int) (numAngle * (alpha + 2 * M_PI) / (4 * M_PI));
                uint accIndex = corrI * numAngle + alpha_index;
                accumulatorPtr[accIndex]++;
            });

        }
        //int sum=0;
        //TODO 用更好的方法找多个极大值
        for (uint k = 0; k < numRefPoints; ++k) {
            for (int j = 0; j < numAngle; ++j) {
                const uint accInd = k * numAngle + j;
                //sum +=accumulatorPtr[accInd];
                const uint accVal = accumulatorPtr[accInd];
                if (accVal > maxVotes) {
                    maxVotes = accVal;
                    refIndMax = k;
                    alphaIndMax = j;
                }
#if !defined (_OPENMP)
                accumulatorPtr[accInd] = 0;
#endif
            }
        }
        //std::cout<<maxVotes<<"--"<<sum/numRefPoints/numAngle<<std::endl;
        if (refIndMax == 0 && alphaIndMax == 0) {
            continue;
        }
        numPosesAdded++;
        // invert Tsg : Luckily rotation is orthogonal:
        // Inverse = Transpose.
        // We are not required to invert

        Eigen::Vector3d tInv, tmg;
        Eigen::Matrix3d Rmg;
        RInv = Rsg.transpose();
        tInv = -RInv * tsg;

        Eigen::Matrix4d TsgInv;
        rtToPose(RInv, tInv, TsgInv);

        // TODO : Compute pose
        const Eigen::Vector3d pMax = scenePointPtr->points_[refIndMax];
        const Eigen::Vector3d nMax = scenePointPtr->normals_[refIndMax];

        computeTransformRT(pMax, nMax, Rmg, tmg);

        Eigen::Matrix4d Tmg;
        rtToPose(Rmg, tmg, Tmg);

        // convert alpha_index to alpha
        int alpha_index = alphaIndMax;
        double alpha = (alpha_index * (4 * M_PI)) / numAngle - 2 * M_PI;

        // Equation 2:
        Eigen::Matrix4d Talpha;
        Eigen::Matrix3d R;
        Eigen::Vector3d t(0, 0, 0);
        getUnitXRotation(alpha, R);
        rtToPose(R, t, Talpha);

        Eigen::Matrix4d rawPose = TsgInv * (Talpha * Tmg);

        Pose_3D pose;
        pose.alpha = alpha;
        pose.modelIndex = refIndMax;
        pose.numVotes = maxVotes;
        pose.pose = rawPose;

#if defined (_OPENMP)
#pragma omp critical
#endif
        {
            poseList.push_back(pose);
        }

/*#if defined (_OPENMP)
        free(accumulator);
#endif*/
    }

    // TODO : Make the parameters relative if not arguments.
    //double MinMatchScore = 0.5;

    std::cout << "numPoses = " << numPosesAdded << std::endl;

    clusterPoses(poseList, numPosesAdded, results);
    std::cout << "time uesd:" << QDateTime::currentDateTime().toMSecsSinceEpoch() - start << "ms" << std::endl;
    //results = poseList;
}
