#ifndef SURFACEMATCH_H
#define SURFACEMATCH_H

#include <iostream>
#include <QObject>
#include <QScopedPointer>
#include <QCoreApplication>
#include <QFuture>
#include <QMutex>
#include <QVariant>
#include <halconcpp/HalconCpp.h>
#include <hdevengine/HDevEngineCpp.h>

using namespace HalconCpp;
Q_DECLARE_METATYPE(HTuple);

class surfaceMatch : public QObject {
Q_OBJECT
public:
    explicit surfaceMatch(QObject *parent = nullptr);

    void train(QFuture<void> &f, HTuple &hv_ObjectModel3D, double RelSamplingDistance);

    double match(HTuple &hv_ObjectScene3D, double RelSamplingDistance, double KeyPointFraction, HTuple &Pose);

    void changeStatus(bool isStart);
    QMutex readMutex;
    HTuple hv_ObjectScene3D;


private:
    HTuple hv_ObjectModel3D, hv_Status, hv_WindowHandle;
    HTuple hv_SurfaceModelID,hv_ObjectScene3DClone,hv_temp, hv_Status1;
    HTuple hv_T0, hv_Pose, hv_Score, hv_SurfaceMatchingResultID;
    HTuple hv_Pose1, hv_Score1, hv_SurfaceMatchingResultID1;
    HTuple hv_T1, hv_TimeForMatching, hv_ObjectModel3DResult;
    HTuple hv_ObjectModel3DRigidTrans, hv_Message, hv_ScoreString;
    HTuple hv_NumResult, hv_Colors, hv_Indices, hv_PoseOut;
    bool isTrained,isStartMatch;
    QVariant poseVar;

signals:
    void registerComplete(QVariant pose, double Score);

protected slots:
    void continueMatch(double RelSamplingDistance, double KeyPointFraction);

};

extern void visualize_object_model_3d(bool isClickable, HTuple hv_WindowHandle, HTuple hv_ObjectModel3D,
                                      HTuple hv_CamParam, HTuple hv_PoseIn, HTuple hv_GenParamName,
                                      HTuple hv_GenParamValue,
                                      HTuple hv_Title, HTuple hv_Label, HTuple hv_Information, HTuple *hv_PoseOut);

extern void set_display_font(HTuple hv_WindowHandle, HTuple hv_Size, HTuple hv_Font, HTuple hv_Bold,
                             HTuple hv_Slant);

#endif // SURFACEMATCH_H
