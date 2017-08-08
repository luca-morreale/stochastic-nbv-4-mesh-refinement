#include <meshac/ResidualPointAccuracyModel.hpp>

namespace meshac {

    ResidualPointAccuracyModel::ResidualPointAccuracyModel(SfMData &data) : PointAccuracyModel(data.points_)
    {
        this->fileList = data.camerasPaths_;
        this->cameras = data.camerasList_;
        this->camObservations = data.camViewing2DPoint_;
        this->point3DTo2DThroughCam = data.point3DTo2DThroughCam_;
    }

    ResidualPointAccuracyModel::~ResidualPointAccuracyModel()
    {
        this->fileList.clear();
        this->cameras.clear();
        this->camObservations.clear();
        this->point3DTo2DThroughCam.clear();
    }


    EigMatrixList ResidualPointAccuracyModel::getAccuracyForPointByImage(int index3DPoint)
    {
        EigMatrixList uncertainties;
        CamToPointMap mapping = point3DTo2DThroughCam[index3DPoint];
        // FIXME can do this in parallel
        for (CamPointPair cameraObsPair : mapping) {

            EigMatrix uncertainty = computeResidual(cameraObsPair, getPoints()[index3DPoint]);
            uncertainties.push_back(uncertainty);
        }
        return uncertainties;
    }

    EigMatrix ResidualPointAccuracyModel::computeResidual(CamPointPair &camToPoint, GLMVec3 &point)
    {
        int camIndex = camToPoint.first;
        GLMVec2 point2D = camToPoint.second;
        EigMatrix matResidual = EigZeros(3);
        CameraMatrix P = getCameraMatrix(camIndex);

        GLMVec2 projectedPoint = getProjectedPoint(point, P);
        GLMVec2 residual = projectedPoint - point2D;

        matResidual(0,0) = residual.x;
        matResidual(1,1) = residual.y;

        return matResidual;
    }    

    GLMVec2ArrayList ResidualPointAccuracyModel::getCamObservations()
    {
        return camObservations;
    }

    ListMappingGLMVec2 ResidualPointAccuracyModel::getMapping3DTo2DThroughCam()
    {
        return point3DTo2DThroughCam;
    }

    StringList ResidualPointAccuracyModel::getFileList()
    {
        return this->fileList;
    }

    void ResidualPointAccuracyModel::setFileList(StringList &fileList)
    {
        this->fileList = fileList;
    }

    CameraList ResidualPointAccuracyModel::getCameras()
    {
        return this->cameras;
    }

    CameraMatrix ResidualPointAccuracyModel::getCameraMatrix(int camIndex)
    {
        return this->cameras[camIndex].cameraMatrix;
    }

    CameraMatrixList ResidualPointAccuracyModel::getCamerasMatrix()
    {
        CameraMatrixList list;
        for (int i = 0; i < cameras.size(); i++) {
            list.push_back(getCameraMatrix(i));
        }

        return list;
    }

    void ResidualPointAccuracyModel::setCameras(CameraList &cameras)
    {
        this->cameras = cameras;
    }    

    void ResidualPointAccuracyModel::appendCamera(CameraType &cam)
    {
        this->cameras.push_back(cam);
        this->camObservations.push_back(GLMVec2List());
    }

    void ResidualPointAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations)  
    {
        this->camObservations = newCamObservations;
    }

    void ResidualPointAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
    {
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, "of camera to set the camera's observation.");
    }

    void ResidualPointAccuracyModel::updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
    {
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, "of camera to updated the camera's observation.");
    }


    void ResidualPointAccuracyModel::setMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    void ResidualPointAccuracyModel::updateMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    /*
     * Protected methods that provide a general way to update lists
     */
    void ResidualPointAccuracyModel::camObservationGeneralUpdate(IntList &indexs, GLMVec2ArrayList &list, GLMVec2ArrayList &targetList, std::string errorMsg)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                throw InvalidUpdateException("Invalid index(" + std::to_string(i) + ") " + errorMsg);
            } else {
                targetList[i].insert(targetList[i].begin(), list[i].begin(), list[i].end());
            }
        }
    }

    void ResidualPointAccuracyModel::mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                targetList.resize(i, std::map<int, GLMVec2>());
            }
            targetList[i].insert(list[i].begin(), list[i].end());
        }
    }


} // namespace meshac
