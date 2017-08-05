#include <meshac/InvertedResidualPointAccuracyModel.hpp>

namespace meshac {

    InvertedResidualPointAccuracyModel::InvertedResidualPointAccuracyModel(SfMData &data) : ResidualPointAccuracyModel(data)
    { /*    */ }
    InvertedResidualPointAccuracyModel::~InvertedResidualPointAccuracyModel()
    { /*    */ }

    EigMatrix InvertedResidualPointAccuracyModel::computeResidual(CamPointPair &camToPoint, GLMVec3 &point)
    {
        int camIndex = camToPoint.first;
        GLMVec2 point2D = camToPoint.second;
        EigMatrix matResidual = EigZeros(3);
        CameraMatrix P = getCameraMatrix(camIndex);

        GLMVec3 projectedPoint = getRetroProjection(P, point2D);
        GLMVec3 residual = projectedPoint - point;

        matResidual(0,0) = residual.x;
        matResidual(1,1) = residual.y;
        matResidual(2,2) = residual.z;

        return matResidual;
    }

    GLMVec3 InvertedResidualPointAccuracyModel::getRetroProjection(CameraMatrix &P, GLMVec2 &point2D)
    {
        GLMVec4 projectedPoint = glm::inverse(glm::transpose(P) * P) * glm::transpose(P) * GLMVec4(point2D, 1.0f, 0.0f);
        projectedPoint /= projectedPoint.w;

        return GLMVec3(projectedPoint.x, projectedPoint.y, projectedPoint.z);
    }

} // namespace meshac
