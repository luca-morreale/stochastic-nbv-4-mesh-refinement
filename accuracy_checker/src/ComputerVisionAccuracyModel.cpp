
#include <meshac/ComputerVisionAccuracyModel.hpp>

namespace meshac {

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(StringList &fileList, CameraMatrixList &cameras, 
                                    GLMVec2ArrayList &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize) 
                                    : PhotogrammetristAccuracyModel(fileList, cameras, camObservations, point3DTo2DThroughCam, pixelSize)
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(StringList &fileList, CameraList &cameras, 
                                    GLMVec2ArrayList &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize)
                                    : PhotogrammetristAccuracyModel(fileList, cameras, camObservations, point3DTo2DThroughCam, pixelSize)
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(SfMData &data, DoublePair &pixelSize) 
                                    : PhotogrammetristAccuracyModel(data, pixelSize) 
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize) 
                                    : PhotogrammetristAccuracyModel(data, pathPrefix, pixelSize) 
    { /*    */ }


    EigMatrixList ComputerVisionAccuracyModel::getAccuracyForPoint(int index3DPoint)
    {
        EigMatrixList uncertaintyMatrix;
        ListMappingGLMVec2 point3DTo2DThroughCam = this->getMapping3DTo2DThroughCam();
        CameraMatrixList cameras = this->getCameras();
        ImagePointVarianceEstimatorPtr varianceEstimator = this->getVarianceEstimator();

        for (auto cameraObsPair : point3DTo2DThroughCam[index3DPoint]) {
            
            GLMVec2 point2D = cameraObsPair.second;
            EigMatrixList pointMatrixList = varianceEstimator->collectPointVarianceMatrix(point2D, cameraObsPair.first);
            
            EigMatrix jacobian = this->computeJacobian(cameras[cameraObsPair.first], point2D);

            for (EigMatrix mat : pointMatrixList) {
                EigMatrix cov = EigZeros(jacobian.cols());
                cov.block(0, 0, jacobian.cols(), jacobian.cols()) += mat;
                uncertaintyMatrix.push_back(jacobian * mat * jacobian.transpose());
            }
        }

        return uncertaintyMatrix;
    }


    /*
     * Computes the Jacobian of the photogrammetrist's function wrt x and y.
     */
    EigMatrix ComputerVisionAccuracyModel::computeJacobian(CameraMatrix &cam, GLMVec2 &point)
    {
        GLMVec2 pointXh = point + GLMVec2(1.0f, 0.f);
        GLMVec2 pointYh = point + GLMVec2(0.f, 1.0f);

        EigVector original = this->evaluateFunctionIn(cam, point);
        EigVector xh = this->evaluateFunctionIn(cam, pointXh);
        EigVector yh = this->evaluateFunctionIn(cam, pointYh);

        EigVector Jx = xh - original;
        EigVector Jy = yh - original;
        EigVector padding = EigZeros(Jx.rows(), 1);

        EigMatrix jacobian(Jx.rows(), 3);
        jacobian << Jx, Jy, padding;

        return jacobian;
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

        return A.transpose() * (A * A.transpose()).inverse() * b;    // in reality this is a column vector
    }

} // namespace meshac

