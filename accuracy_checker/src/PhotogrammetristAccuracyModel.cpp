
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(StringList &fileList, CameraMatrixList &cameras, GLMVec2ArrayList &camObservations,
                                                                    ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize)
    {
        this->fileList = fileList;
        this->cameras = cameras;
        this->camObservations = camObservations;
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;

        this->initMembers(pixelSize);
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(StringList &fileList, CameraList &cameras, GLMVec2ArrayList &camObservations,
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
    // EigMatrixList PhotogrammetristAccuracyModel::getAccuracyForPointByImage(int index3DPoint)
    // {
    //     IntDoubleListMap variances;
    //     IntEigMatrixListMap jacobianList;

    //     CamToPointMap mapping = point3DTo2DThroughCam[index3DPoint];
    //     for (CamPointPair cameraObsPair : mapping) {
            
    //         GLMVec2 point2D = cameraObsPair.second;

    //         EigMatrix pointVariance = this->varianceEstimator->estimateVarianceMatrixForPoint(point2D, cameraObsPair.first);
    //         EigMatrix jacobian = this->computeJacobian(this->cameras[cameraObsPair.first], point2D);  // 3x2 vector

    //         this->updateVariancesList(variances[cameraObsPair.first], pointVariance, jacobianList[cameraObsPair.first], jacobian);
    //     }

    //     EigMatrixList pointCovariance = generateDiagonalMatrix(variances);
    //     EigMatrixList jacobian = juxtaposeMatrixs(jacobianList, pointCovariance);

    //     return computesProducts(jacobian, pointCovariance);
    // }

    EigMatrixList PhotogrammetristAccuracyModel::getAccuracyForPointByImage(int index3DPoint)
    {
        EigMatrixList uncertainties;
        CamToPointMap mapping = point3DTo2DThroughCam[index3DPoint];
        for (CamPointPair cameraObsPair : mapping) {        // get a camera-point for each projection

            // get cross ratio tuple set
            // for each tuple estimate jacobian
            // for each tuple estimate jacobian * variance * jacobian^T
            // do average of each matrix obtained above
            // return matrix list
            
            EigMatrix uncertainty = getAccuracyForPointInImage(cameraObsPair);
            uncertainties.push_back(uncertainty);
        }
        return uncertainties;
    }

    EigMatrix PhotogrammetristAccuracyModel::getAccuracyForPointInImage(CamPointPair &cameraObsPair)
    {
        CrossRatioTupleSetVariance pointVariance = this->varianceEstimator->estimateVarianceMatrixForPoint(cameraObsPair.second, cameraObsPair.first);
        CrossRatioTupleSet tuples = pointVariance.first;
        EigMatrix variance = replicateVarianceForTuple(pointVariance.second);

        EigMatrixList uncertainties;
        for (auto tuple : tuples) {
            EigMatrix jacobian = this->computeJacobian(tuple, this->cameras[cameraObsPair.first]);  // 3x(2*4) vector
            uncertainties.push_back(jacobian * variance * jacobian.transpose());
        }
        return average(uncertainties);
    }

    EigMatrix PhotogrammetristAccuracyModel::replicateVarianceForTuple(EigMatrix &singleVariance)
    {
        EigMatrix variance = EigZeros(4 * 2);
        variance(0,0) = singleVariance(0, 0);
        variance(1,1) = singleVariance(1, 1);
        variance(2,2) = singleVariance(0, 0);
        variance(3,3) = singleVariance(1, 1);
        variance(4,4) = singleVariance(0, 0);
        variance(3,3) = singleVariance(1, 1);
        variance(6,6) = singleVariance(0, 0);
        variance(7,7) = singleVariance(1, 1);

        return variance;
    }

    /*
     * Computes the Jacobian of the photogrammetrist's function wrt x and y.
     */
    EigMatrix PhotogrammetristAccuracyModel::computeJacobian(CrossRatioTuple &tuple, CameraMatrix &cam)
    {
        EigMatrixList jacobians;
        for (auto point : tuple.getPoints()) {
            GLMVec2 pointXh = point + GLMVec2(1.0f, 0.f);
            GLMVec2 pointYh = point + GLMVec2(0.f, 1.0f);

            EigVector original = this->evaluateFunctionIn(cam, point);

            EigVector Jx = this->computeSingleJacobianFor(original, cam, pointXh);
            EigVector Jy = this->computeSingleJacobianFor(original, cam, pointYh);

            EigMatrix singleJacobian(Jx.rows(), 2);
            singleJacobian << Jx, Jy;
            jacobians.push_back(singleJacobian);
        }
        return juxtaposeMatrixs(jacobians);
    }

    EigVector PhotogrammetristAccuracyModel::computeSingleJacobianFor(EigVector &original, CameraMatrix &cam, GLMVec2 &pointH)
    {
        return this->evaluateFunctionIn(cam, pointH) - original;
    }

    EigMatrixList PhotogrammetristAccuracyModel::computesProducts(EigMatrixList &jacobian, EigMatrixList &pointCovariance)
    {
        EigMatrixList results;
        #pragma omp parallel for
        for (int i = 0; i < jacobian.size(); i++) {
            EigMatrix mat = jacobian[i] * pointCovariance[i] * jacobian[i].transpose();

            #pragma omp critical
            results.push_back(mat);

        }
        return results;
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

    void PhotogrammetristAccuracyModel::iterativeEstimationOfCovariance(EigMatrixList &destList, EigMatrixList &pointMatrixList, EigMatrix &jacobian)
    {
        for (EigMatrix mat : pointMatrixList) {
            destList.push_back(jacobian * mat * jacobian.transpose());
        }
    }

    void PhotogrammetristAccuracyModel::updateVariancesList(DoubleList &varianesList, EigMatrix &varianceMat, EigMatrixList &jacobianList, EigMatrix &jacobianMat)
    {
        appendMatrixDiagonalToVector(varianceMat, varianesList);
        jacobianList.push_back(jacobianMat.replicate(1, (int)varianceMat.rows()/2));
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

    GLMVec2ArrayList PhotogrammetristAccuracyModel::getCamObservations()
    {
        return camObservations;
    }

    ListMappingGLMVec2 PhotogrammetristAccuracyModel::getMapping3DTo2DThroughCam()
    {
        return point3DTo2DThroughCam;
    }

    StringList PhotogrammetristAccuracyModel::getFileList()
    {
        return this->fileList;
    }

    CameraMatrixList PhotogrammetristAccuracyModel::getCameras()
    {
        return this->cameras;
    }

    ImagePointVarianceEstimatorPtr PhotogrammetristAccuracyModel::getVarianceEstimator()
    {
        return this->varianceEstimator;
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
        this->camObservations.push_back(GLMVec2List());
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations)  
    {
        this->camObservations = newCamObservations;
        this->varianceEstimator->setCameraObservations(newCamObservations);
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
    {
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, "of camera to set the camera's observation.");
        this->varianceEstimator->setCameraObservations(newCamObservations, camIndexs);
    }

    void PhotogrammetristAccuracyModel::updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
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
    void PhotogrammetristAccuracyModel::camObservationGeneralUpdate(IntList &indexs, GLMVec2ArrayList &list, GLMVec2ArrayList &targetList, std::string errorMsg)
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
