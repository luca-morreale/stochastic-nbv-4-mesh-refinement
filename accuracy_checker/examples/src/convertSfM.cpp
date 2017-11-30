#include <convertSfM.hpp>

SfMData convertSfMData(meshac::SfMData &src)
{
    SfMData dest;
    dest.numPoints_ = src.numPoints_;
    dest.numCameras_ = src.numCameras_;

    dest.points_ = src.points_;
    dest.camerasList_ = convertList(src.camerasList_);
    dest.camerasPaths_ = src.camerasPaths_;

    dest.camViewingPointN_ = src.camViewingPointN_;
    dest.pointsVisibleFromCamN_ = src.pointsVisibleFromCamN_;
    dest.point2DoncamViewingPoint_ = src.point2DoncamViewingPoint_;

    dest.imageWidth_ = src.imageWidth_;
    dest.imageHeight_ = src.imageHeight_;
    return dest;
}

std::vector<CameraType> convertList(meshac::CameraList &cameras)
{
    std::vector<CameraType> dest;
    for (meshac::CameraType camera : cameras) {
        dest.push_back(convertCameraType(camera));
    }
    return dest;
}

CameraType convertCameraType(meshac::CameraType &src)
{
    CameraType dest;
    dest.idCam = src.idCam;
    dest.idReconstruction = src.idReconstruction;

    dest.intrinsics = src.intrinsics;
    dest.rotation = src.rotation;
    dest.translation = src.translation;
    dest.cameraMatrix = src.cameraMatrix;
    dest.center = src.center;
    dest.mvp = src.mvp;

    dest.pathImage = src.pathImage;

    dest.imageHeight = src.imageHeight;
    dest.imageWidth = src.imageWidth;
    return dest;
}
