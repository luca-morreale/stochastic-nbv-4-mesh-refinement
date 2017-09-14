
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(SfMData &data, DoublePair &pixelSize)
                                    : ResidualPointAccuracyModel(data)
    {
        this->initMembers(pixelSize);
    }

    PhotogrammetristAccuracyModel::PhotogrammetristAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize)
                                    : ResidualPointAccuracyModel(data)
    {        
        this->fixImagesPath(pathPrefix);
        this->initMembers(pixelSize);
    }
    
    PhotogrammetristAccuracyModel::~PhotogrammetristAccuracyModel() 
    {
        delete this->varianceEstimator;
    }


    /*
     * Protected method to initialize all members.
     */
    void PhotogrammetristAccuracyModel::initMembers(DoublePair &pixelSize)
    {
        auto fileList = this->getFileList();
        auto camObservations = this->getCamObservations();
        this->varianceEstimator = new ImagePointVarianceEstimator(fileList, camObservations, pixelSize);
    }

    void PhotogrammetristAccuracyModel::fixImagesPath(std::string &pathPrefix)
    {
        StringList fileList = this->getFileList();
        std::for_each(fileList.begin(), fileList.end(), [pathPrefix](std::string &path) { path.insert(0, pathPrefix); } );
        this->setFileList(fileList);
    }

    /*
     * Estimates the uncertainties for the 3D point.
     */
    EigMatrixList PhotogrammetristAccuracyModel::getAccuracyForPointByImage(int index3DPoint)
    {
        EigMatrixList uncertainties;
        CamToPointMap mapping = getMapping3DTo2DThroughCam()[index3DPoint];
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
            CameraMatrix P = this->getCameraMatrix(cameraObsPair.first);
            EigMatrix jacobian = this->computeJacobian(tuple, P);  // 3x(2*4) vector
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
        GLMVec2List points = tuple.getPoints();
        EigMatrixList jacobians(points.size());

        #pragma omp parallel for
        for (int i = 0; i < points.size(); i++) {
            GLMVec2 pointXh = points[i] + GLMVec2(1.0f, 0.f);
            GLMVec2 pointYh = points[i] + GLMVec2(0.f, 1.0f);

            EigVector original = this->evaluateFunctionIn(cam, points[i]);

            EigVector Jx = this->computeSingleJacobianFor(original, cam, pointXh);
            EigVector Jy = this->computeSingleJacobianFor(original, cam, pointYh);

            EigMatrix singleJacobian(Jx.rows(), 2);
            singleJacobian << Jx, Jy;
            jacobians[i] = singleJacobian;
        }
        return juxtaposeMatrixs(jacobians);
    }

    EigVector PhotogrammetristAccuracyModel::computeSingleJacobianFor(EigVector &original, CameraMatrix &cam, GLMVec2 &pointH)
    {
        return this->evaluateFunctionIn(cam, pointH) - original;
    }

    EigMatrixList PhotogrammetristAccuracyModel::computesProducts(EigMatrixList &jacobian, EigMatrixList &pointCovariance)
    {
        EigMatrixList results(jacobian.size());

        #pragma omp parallel for
        for (int i = 0; i < jacobian.size(); i++) {
            EigMatrix mat = jacobian[i] * pointCovariance[i] * jacobian[i].transpose();
            results[i] = mat;
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
        destList.assign(pointMatrixList.size(), EigMatrix());

        #pragma omp parallel for
        for (int i = 0; i < pointMatrixList.size(); i++) {
            destList[i] = jacobian * pointMatrixList[i] * jacobian.transpose();
        }

    }

    void PhotogrammetristAccuracyModel::updateVariancesList(DoubleList &varianesList, EigMatrix &varianceMat, EigMatrixList &jacobianList, EigMatrix &jacobianMat)
    {
        appendMatrixDiagonalToVector(varianceMat, varianesList);
        jacobianList.push_back(jacobianMat.replicate(1, (int)varianceMat.rows()/2));
    }

    ImagePointVarianceEstimatorPtr PhotogrammetristAccuracyModel::getVarianceEstimator()
    {
        return this->varianceEstimator;
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations)  
    {
        super::setCameraObservations(newCamObservations);
        this->varianceEstimator->setCameraObservations(newCamObservations);
    }

    void PhotogrammetristAccuracyModel::setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
    {
        super::setCameraObservations(newCamObservations, camIndexs);
        this->varianceEstimator->setCameraObservations(newCamObservations, camIndexs);
    }

    void PhotogrammetristAccuracyModel::updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs)  
    {
        super::updateCameraObservations(newCamObservations, camIndexs);
        this->varianceEstimator->updateCameraObservations(newCamObservations, camIndexs);
    }


} // namespace meshac
