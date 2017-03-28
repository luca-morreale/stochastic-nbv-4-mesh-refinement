
#include <meshac/ComputerVisionAccuracyModel.hpp>

namespace meshac {

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(StringList &fileList, CameraMatrixList &cameras, 
                                    GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam) 
                                    : PhotogrammetristAccuracyModel(fileList, cameras, camObservations, point3DTo2DThroughCam)
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(StringList &fileList, CameraList &cameras, 
                                    GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam)
                                    : PhotogrammetristAccuracyModel(fileList, cameras, camObservations, point3DTo2DThroughCam)
    { /*    */ }

    ComputerVisionAccuracyModel::ComputerVisionAccuracyModel(SfMData &data) : PhotogrammetristAccuracyModel(data) 
    { /*    */ }

    /*
     * Evaluates the photogrammetrist's function in the given point with the given camera.
     */
    EigVector4 ComputerVisionAccuracyModel::evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point)
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

