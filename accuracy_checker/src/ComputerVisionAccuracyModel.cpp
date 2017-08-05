
#include <meshac/ComputerVisionAccuracyModel.hpp>

namespace meshac {

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(SfMData &data, DoublePair &pixelSize) 
                                    : PhotogrammetristAccuracyModel(data, pixelSize) 
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize) 
                                    : PhotogrammetristAccuracyModel(data, pathPrefix, pixelSize) 
    { /*    */ }

    EigMatrix ComputerVisionAccuracyModel::getAccuracyForPointInImage(CamPointPair &cameraObsPair)
    {
        CrossRatioTupleSetVariance pointVariance = this->getVarianceEstimator()->estimateVarianceMatrixForPoint(cameraObsPair.second, cameraObsPair.first);
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

    EigMatrix ComputerVisionAccuracyModel::replicateVarianceForTuple(EigMatrix &singleVariance)
    {
        EigMatrix variance = EigZeros(4 * 3);
        variance(0,0) = singleVariance(0, 0);
        variance(1,1) = singleVariance(1, 1);

        variance(3,3) = singleVariance(0, 0);
        variance(4,4) = singleVariance(1, 1);

        variance(6,6) = singleVariance(0, 0);
        variance(7,7) = singleVariance(1, 1);

        variance(9,9) = singleVariance(0, 0);
        variance(10,10) = singleVariance(1, 1);
        return variance;
    }
    

    /*
     * Computes the Jacobian of the photogrammetrist's function wrt x and y.
     */
    EigMatrix ComputerVisionAccuracyModel::computeJacobian(CrossRatioTuple &tuple, CameraMatrix &cam)
    {
        EigMatrixList jacobians;
        for (auto point : tuple.getPoints()) {
            GLMVec2 pointXh = point + GLMVec2(1.0f, 0.f);
            GLMVec2 pointYh = point + GLMVec2(0.f, 1.0f);

            EigVector original = this->evaluateFunctionIn(cam, point);

            EigVector Jx = this->computeSingleJacobianFor(original, cam, pointXh);
            EigVector Jy = this->computeSingleJacobianFor(original, cam, pointYh);
            EigVector padding = EigZeros(Jx.rows(), 1);

            EigMatrix singleJacobian(Jx.rows(), 3);
            singleJacobian << Jx, Jy, padding;
            jacobians.push_back(singleJacobian);
        }
        return juxtaposeMatrixs(jacobians);
    }
    
    /*
     * Evaluates the photogrammetrist's function in the given point with the given camera.
     */
    EigVector ComputerVisionAccuracyModel::evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point)
    {
        EigCameraMatrix A = EigZeros(3, 4); // A = [x*P31-P11  x*P32-P12  x*P33-P13 -P14; y*P31-P21  y*P32-P22  y*P33-P23 -P24; 0 0 0 1];
        EigVector3 b = EigOnes(3, 1); // b = [-x*P34; -y*P34; 1];
        
        A(0,0) = cam[2][1] * point[0] - cam[0][0];
        A(0,1) = cam[2][1] * point[0] - cam[0][1];
        A(0,2) = cam[2][2] * point[0] - cam[0][2];
        A(0,3) = -cam[0][3];

        A(1,0) = cam[2][1] * point[1] - cam[1][0];
        A(1,1) = cam[2][1] * point[1] - cam[1][1];
        A(1,2) = cam[2][2] * point[1] - cam[1][2];
        A(1,3) = -cam[1][3];

        A(2,3) = 1;

        b(0) = -point[0] * cam[2][3];
        b(1) = -point[1] * cam[2][3];

        return A.transpose() * (A * A.transpose()).inverse() * b;    // this should be a column vector
    }

    void ComputerVisionAccuracyModel::iterativeEstimationOfCovariance(EigMatrixList &destList, EigMatrixList &pointMatrixList, EigMatrix &jacobian)
    {
        for (EigMatrix mat : pointMatrixList) {
            EigMatrix cov = EigIdentity(jacobian.cols());
            cov.block(0, 0, mat.cols(), mat.cols()) += mat;
            destList.push_back(jacobian * cov * jacobian.transpose());
        }
    }

    void ComputerVisionAccuracyModel::updateVariancesList(DoubleList &varianesList, EigMatrix &varianceMat, EigMatrixList &jacobianList, EigMatrix &jacobianMat)
    {
        EigMatrix cov = EigZeros(jacobianMat.cols());
        cov.block(0, 0, jacobianMat.cols(), jacobianMat.cols()) += varianceMat;
        super::updateVariancesList(varianesList, cov, jacobianList, jacobianMat);
    }

} // namespace meshac

