
#include <PhotogrammetristAccuracyModel.hpp>

namespace meshac {

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(GLMListVec3 points3D, CameraMatrixList cameras, 
                                GLMListArrayVec2 camObservations, ListMappingGLMVec2 point3DTo2DThroughCam, int obsWidth, int obsHeight)
    {
        this->cameras = cameras;
        this->camObservations = camObservations;
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;
        this->obsWidth = obsWidth;
        this->obsHeight = obsHeight;

        this->initMembers();
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(GLMListVec3 points3D, CameraList cameras, 
                                GLMListArrayVec2 camObservations, ListMappingGLMVec2 point3DTo2DThroughCam, int obsWidth, int obsHeight)
    {
        this->cameras = this->extractCameraMatrix(cameras);
        this->camObservations = camObservations;
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;
        this->obsWidth = obsWidth;
        this->obsHeight = obsHeight;

        this->initMembers();
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(SfMData data)
    {
        this->cameras = this->extractCameraMatrix(data.camerasList_);
        this->camObservations = data.camViewing2DPoint_;
        this->point3DTo2DThroughCam = data.point3DTo2DThroughCam_;
        this->obsWidth = data.imageWidth_;
        this->obsHeight = data.imageHeight_;

        this->initMembers();
    }
    
    PhotogrammetristAccuracyModel::~PhotogrammetristAccuracyModel() 
    {
        delete this->tuplesGenerator;
        delete this->varianceEstimator;

        this->cameras.clear();
        this->camObservations.clear();
        this->point3DTo2DThroughCam.clear();
    }


    /*
     * Protected method to initialize all members.
     */
    void PhotogrammetristAccuracyModel::initMembers()
    {
        this->tuplesGenerator = new meshac::CRTuplesGenerator(this->camObservations, this->obsWidth, this->obsHeight);
        this->varianceEstimator = new ImagePointVarianceEstimator(this->tuplesGenerator->determineTupleOfFourPointsForAllCam());
    }

    /*
     * Estimates the uncertainties for the 3D point.
     */
    EigMatrixList PhotogrammetristAccuracyModel::getAccuracyForPoint(int index3DPoint)  // TO check dimension of the multiplication
    {
        EigMatrixList uncertaintyMatrix;
        
        #pragma omp parallel for
        for (auto cameraObsPair : point3DTo2DThroughCam[index3DPoint]) {
            
            GLMVec2 point2D = cameraObsPair.second;
            EigMatrix4 pointVariance = this->varianceEstimator->estimateVarianceForPoint(point2D, cameraObsPair.first); // FIX

            EigMatrix jacobian = this->computeJacobian(this->cameras[cameraObsPair.first], point2D);

            #pragma omp critical
            uncertaintyMatrix.push_back(jacobian.transpose() * pointVariance * jacobian);   // FIX
        }

        return uncertaintyMatrix;
    }

    /*
     * Computes the Jacobian of the photogrammetrist's function wrt x and y.
     */
    EigMatrix PhotogrammetristAccuracyModel::computeJacobian(CameraMatrix &cam, GLMVec2 &point)
    {
        GLMVec2 pointXh = point + GLMVec2(this->getXh(), 0.f);
        GLMVec2 pointYh = point + GLMVec2(0.f, this->getYh());

        EigVector4 original = evaluateFunctionIn(cam, point);
        EigVector4 originalXh = evaluateFunctionIn(cam, pointXh);
        EigVector4 originalYh = evaluateFunctionIn(cam, pointYh);

        EigVector4 Jx = (originalXh - original) / xh;
        EigVector4 Jy = (originalYh - original) / yh;

        EigMatrix jacobian;
        jacobian << Jx, Jy;

        return jacobian;
    }

    /*
     * Evaluates the photogrammetrist's function in the given point with the given camera.
     */
    EigVector4 PhotogrammetristAccuracyModel::evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point)
    {
        EigCameraMatrix A = EigZeros(2, 3); // A = [x*P31-P11  x*P32-P12  x*P33-P13; y*P31-P21  y*P32-P22  y*P33-P23];
        EigVector3 b = EigOnes(2, 1); // b = [P14-x*P34; P24-y*P34];
        
        A(0,0) = cam[2][1] * point[0] - cam[0][0];
        A(0,1) = cam[2][1] * point[0] - cam[0][1];
        A(0,2) = cam[2][2] * point[0] - cam[0][2];
        
        A(1,0) = cam[2][1] * point[1] - cam[1][0];
        A(1,1) = cam[2][1] * point[1] - cam[1][1];
        A(1,2) = cam[2][2] * point[1] - cam[1][2];
        

        b(0) = cam[0][3] - point[0] * cam[2][3];
        b(1) = cam[1][3] - point[1] * cam[2][3];
        
        return A.transpose() * (A * A.transpose()).inverse() * b;    // in reality this is a column vector
    }

    /*
     * Computes the required cross ratio's tuples.
     */
    void PhotogrammetristAccuracyModel::computeTuples()
    {
        this->crossratioTupleSet = this->tuplesGenerator->determineTupleOfFourPoints();
    }

    void PhotogrammetristAccuracyModel::computeTuples(int camIndex)
    {
        this->crossratioTupleSet = this->tuplesGenerator->determineTupleOfFourPointsForCam(camIndex);
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

    std::pair<int, int> PhotogrammetristAccuracyModel::getObservationSize()
    {
        return std::make_pair(obsWidth, obsHeight);
    }

    void PhotogrammetristAccuracyModel::appendCamera(CameraMatrix cam)
    {
        this->cameras.push_back(cam);
        this->camObservations.push_back(GLMListVec2());
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(IntList camIndexs, GLMListArrayVec2 newCamObservations)  
    {
        auto tmp = boost::bind(&CRTuplesGenerator::setCamObservations, tuplesGenerator, _1, _2);
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, tmp, "of camera to set the camera's observation.");
    }

    void PhotogrammetristAccuracyModel::updateCameraObservations(IntList camIndexs, GLMListArrayVec2 newCamObservations)  
    {
        auto tmp = boost::bind(&CRTuplesGenerator::updateCamObservations, tuplesGenerator, _1, _2);
        this->camObservationGeneralUpdate(camIndexs, newCamObservations, camObservations, tmp, "of camera to updated the camera's observation.");
    }

    void PhotogrammetristAccuracyModel::setMapping3DTo2DThroughCam(IntList index3DPoints, ListMappingGLMVec2 indexCams)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    void PhotogrammetristAccuracyModel::updateMapping3DTo2DThroughCam(IntList index3DPoints, ListMappingGLMVec2 indexCams)
    {
        this->mappingGeneralUpdate(index3DPoints, indexCams, point3DTo2DThroughCam);
    }

    /*
     * Protected methods that provide a general way to update lists
     */
    void PhotogrammetristAccuracyModel::camObservationGeneralUpdate(IntList &indexs, GLMListArrayVec2 &list, GLMListArrayVec2 &targetList, FunctionTarget tupleUpdater, std::string errorMsg)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                throw InvalidUpdateException("Invalid index(" + std::to_string(i) + ") " + errorMsg);
            } else {
                targetList[i].insert(targetList[i].begin(), list[i].begin(), list[i].end());
                tupleUpdater(targetList[i], i);
            }
        }
    }

    void PhotogrammetristAccuracyModel::mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList)
    {
        for (int i : indexs) {
            if (i > list.size()) {
                targetList.push_back(std::map<int, GLMVec2>());
            } else {
                targetList[i].insert(list[i].begin(), list[i].end());
            }
        }
    }


    /*
     * Getter and Setter for CrossRatio Tuples' Generator.
     */
    void PhotogrammetristAccuracyModel::setTupleGenerator(CRTuplesGeneratorPtr generator)
    {
        this->tuplesGenerator = generator;
    }

    CRTuplesGeneratorPtr PhotogrammetristAccuracyModel::getTuplesGenerator()
    {
        return tuplesGenerator;
    }


    /*
     * Setters for the information of the different cameras and views.
     */
    void PhotogrammetristAccuracyModel::setCameras(CameraMatrixList cameras)
    {
        this->cameras = cameras;
    }
    void PhotogrammetristAccuracyModel::setCamObservations(GLMListArrayVec2 camObservations)
    {
        this->camObservations = camObservations;
    }

    void PhotogrammetristAccuracyModel::setVisibilityOfPoints(ListMappingGLMVec2 point3DTo2DThroughCam)
    {
        this->point3DTo2DThroughCam = point3DTo2DThroughCam;
    }

    float PhotogrammetristAccuracyModel::getXh()
    {
        return xh;
    }

    float PhotogrammetristAccuracyModel::getYh()
    {
        return yh;
    }

} // namespace meshac
