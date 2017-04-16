
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(StringList &fileList, CameraMatrixList &cameras, GLMListArrayVec2 &camObservations,
                                                                    ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize)
    {
        this->fileList = fileList;
        this->cameras = cameras;
        this->camObservations = camObservations;
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;

        this->initMembers(pixelSize);
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(StringList &fileList, CameraList &cameras, GLMListArrayVec2 &camObservations,
                                                                    ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize)
    {
        this->fileList = fileList;
        this->cameras = this->extractCameraMatrix(cameras);
        this->camObservations = camObservations;
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;

        this->initMembers(pixelSize);
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(SfMData &data, DoublePair &pixelSize)
    {
        this->fileList = data.camerasPaths_;
        this->cameras = this->extractCameraMatrix(data.camerasList_);
        this->camObservations = data.camViewing2DPoint_;
        this->point3DTo2DThroughCam = data.point3DTo2DThroughCam_;

        this->initMembers(pixelSize);
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize)
    {
        this->fileList = data.camerasPaths_;
        this->cameras = this->extractCameraMatrix(data.camerasList_);
        this->camObservations = data.camViewing2DPoint_;
        this->point3DTo2DThroughCam = data.point3DTo2DThroughCam_;
        
        this->fixImagesPath(pathPrefix);
        this->initMembers(pixelSize);
    }
    
    PhotogrammetristAccuracyModel::~PhotogrammetristAccuracyModel() 
    {
        delete this->varianceEstimator;

        this->fileList.clear();
        this->cameras.clear();
        this->camObservations.clear();
        this->point3DTo2DThroughCam.clear();
    }


    /*
     * Protected method to initialize all members.
     */
    void PhotogrammetristAccuracyModel::initMembers(DoublePair &pixelSize)
    {
        this->varianceEstimator = new ImagePointVarianceEstimator(this->fileList, this->camObservations, pixelSize);
    }

    void PhotogrammetristAccuracyModel::fixImagesPath(std::string &pathPrefix)
    {
        std::for_each(this->fileList.begin(), this->fileList.end(), [pathPrefix](std::string &path) { path.insert(0, pathPrefix); } );
    }

    /*
     * Estimates the uncertainties for the 3D point.
     */
    EigMatrixList PhotogrammetristAccuracyModel::getAccuracyForPoint(int index3DPoint)
    {
        EigMatrixList uncertaintyMatrix;

        for (auto cameraObsPair : point3DTo2DThroughCam[index3DPoint]) {
            
            
            GLMVec2 point2D = cameraObsPair.second;
            EigMatrixList pointMatrixList = this->varianceEstimator->collectPointVarianceMatrix(point2D, cameraObsPair.first);
            
            EigMatrix jacobian = this->computeJacobian(this->cameras[cameraObsPair.first], point2D);
            
            EigMatrix jacobianMatrix = jacobian.replicate(1, pointMatrixList[0].rows());

            for (EigMatrix mat : pointMatrixList) {
                uncertaintyMatrix.push_back(jacobianMatrix * mat * jacobianMatrix.transpose());
            }
        }

        return uncertaintyMatrix;
    }

    EigMatrix PhotogrammetristAccuracyModel::getCompleteAccuracyForPoint(int index3DPoint)
    {
        DoubleList variances;
        EigMatrixList jacobianList;

        int size = 0;
        
        for (auto cameraObsPair : point3DTo2DThroughCam[index3DPoint]) {
            
            GLMVec2 point2D = cameraObsPair.second;

            EigMatrix pointVariance = this->varianceEstimator->estimateVarianceMatrixForPoint(point2D, cameraObsPair.first);
            EigMatrix jacobian = this->computeJacobian(this->cameras[cameraObsPair.first], point2D);  // 3x2 vector

        
            size += pointVariance.rows();
        
            appendMatrixDiagonalToVector(pointVariance, variances);
            jacobianList.push_back(jacobian.replicate(1, (int)pointVariance.rows()/2));
        }

        EigMatrix jacobian = juxtaposeMatrixs(jacobianList, size);

        return jacobian * generateDiagonalMatrix(variances) * jacobian.transpose();
    }

    /*
     * Computes the Jacobian of the photogrammetrist's function wrt x and y.
     */
    EigMatrix PhotogrammetristAccuracyModel::computeJacobian(CameraMatrix &cam, GLMVec2 &point)
    {
        GLMVec2 pointXh = point + GLMVec2(1.0f, 0.f);
        GLMVec2 pointYh = point + GLMVec2(0.f, 1.0f);

        EigVector original = this->evaluateFunctionIn(cam, point);
        EigVector xh = this->evaluateFunctionIn(cam, pointXh);
        EigVector yh = this->evaluateFunctionIn(cam, pointYh);

        EigVector Jx = xh - original;
        EigVector Jy = yh - original;

        EigMatrix jacobian(Jx.rows(), 2);
        jacobian << Jx, Jy;

        return jacobian;
    }

    /*
     * Evaluates the photogrammetrist's function in the given point with the given camera.
     */
    EigVector PhotogrammetristAccuracyModel::evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point)
    {
        EigMatrix A = EigZeros(2, 3); // A = [x*P31-P11  x*P32-P12  x*P33-P13; y*P31-P21  y*P32-P22  y*P33-P23];
        EigVector2 b = EigOnes(2, 1); // b = [P14-x*P34; P24-y*P34];
        
        A(0,0) = cam[2][0] * point[0] - cam[0][0];
        A(0,1) = cam[2][1] * point[0] - cam[0][1];
        A(0,2) = cam[2][2] * point[0] - cam[0][2];
        
        A(1,0) = cam[2][0] * point[1] - cam[1][0];
        A(1,1) = cam[2][1] * point[1] - cam[1][1];
        A(1,2) = cam[2][2] * point[1] - cam[1][2];
        

        b(0) = cam[0][3] - point[0] * cam[2][3];
        b(1) = cam[1][3] - point[1] * cam[2][3];
        
        return A.transpose() * (A * A.transpose()).inverse() * b;    // this is a column vector
    }


    CameraMatrixList PhotogrammetristAccuracyModel::extractCameraMatrix(CameraList &cameras)
    {
        CameraMatrixList matList;
        for (CameraType cam : cameras) {
            matList.push_back(cam.cameraMatrix);
        }
        return matList;
    }


    CameraMatrixList PhotogrammetristAccuracyModel::getCamerasMatrix()
    {
        return cameras;
    }

    GLMListArrayVec2 PhotogrammetristAccuracyModel::getCamObservations()
    {
        return camObservations;
    }

    ListMappingGLMVec2 PhotogrammetristAccuracyModel::getMapping3DTo2DThroughCam()
    {
        return point3DTo2DThroughCam;
    }

    void PhotogrammetristAccuracyModel::setCameras(CameraMatrixList &cameras)
    {
        this->cameras = cameras;
    }

    void PhotogrammetristAccuracyModel::setCameras(CameraList &cameras)
    {
        this->cameras = this->extractCameraMatrix(cameras);
    }    

    void PhotogrammetristAccuracyModel::appendCamera(CameraMatrix &cam)
    {
        this->cameras.push_back(cam);
        this->camObservations.push_back(GLMListVec2());
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMListArrayVec2 &newCamObservations)  
    {
        this->camObservations = newCamObservations;
        this->varianceEstimator->setCameraObservations(newCamObservations);
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMListArrayVec2 &newCamObservations, IntList &camIndexs)  
    {
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, "of camera to set the camera's observation.");
        this->varianceEstimator->setCameraObservations(newCamObservations, camIndexs);
    }

    void PhotogrammetristAccuracyModel::updateCameraObservations(GLMListArrayVec2 &newCamObservations, IntList &camIndexs)  
    {
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, "of camera to updated the camera's observation.");
        this->varianceEstimator->updateCameraObservations(newCamObservations, camIndexs);
    }


    void PhotogrammetristAccuracyModel::setMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    void PhotogrammetristAccuracyModel::updateMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    /*
     * Protected methods that provide a general way to update lists
     */
    void PhotogrammetristAccuracyModel::camObservationGeneralUpdate(IntList &indexs, GLMListArrayVec2 &list, GLMListArrayVec2 &targetList, std::string errorMsg)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                throw InvalidUpdateException("Invalid index(" + std::to_string(i) + ") " + errorMsg);
            } else {
                targetList[i].insert(targetList[i].begin(), list[i].begin(), list[i].end());
            }
        }
    }

    void PhotogrammetristAccuracyModel::mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                targetList.resize(i, std::map<int, GLMVec2>());
            }
            targetList[i].insert(list[i].begin(), list[i].end());
        }
    }

} // namespace meshac
