#include <meshac/BasicPhotogrammetristAccuracyModel.hpp>

namespace meshac {

    BasicPhotogrammetristAccuracyModel::BasicPhotogrammetristAccuracyModel(SfMData &data) : ResidualPointAccuracyModel(data)
    {
        this->points = data.points_;
        this->camPointMap = data.camViewingPointN_;
    }

    BasicPhotogrammetristAccuracyModel::~BasicPhotogrammetristAccuracyModel()
    { /*    */ }

    EigMatrixList BasicPhotogrammetristAccuracyModel::getAccuracyForPointByImage(int index3DPoint)
    {
        EigMatrixList jacobiansList;
        IntList camsIndex = this->camPointMap[index3DPoint];
        
        #pragma omp parallel for
        for (int i = 0; i < camsIndex.size(); i++) {
            CameraMatrix cam = getCameraMatrix(camsIndex[i]);
            GLMMat3 jacobian = computeJacobian(cam, points[index3DPoint]);
            jacobian = glm::transpose(jacobian) * jacobian;
            jacobian = glm::inverse(jacobian);
            EigMatrix mat = convert(jacobian);

            #pragma omp critical
            jacobiansList.push_back(mat);
        }
        return jacobiansList;
    }

    GLMMat3 BasicPhotogrammetristAccuracyModel::computeJacobian(CameraMatrix &cam, GLMVec3 &point)
    {
        GLMVec3 originalProjection = getStraightProjection(cam, point);

        GLMVec3 pointX = point + GLMVec3(1.0f, 0.0f, 0.0f);
        GLMVec3 pointY = point + GLMVec3(0.0f, 1.0f, 0.0f);
        GLMVec3 pointZ = point + GLMVec3(0.0f, 0.0f, 1.0f);

        GLMVec3 diff1 = getStraightProjection(cam, pointX) - originalProjection;
        GLMVec3 diff2 = getStraightProjection(cam, pointY) - originalProjection;
        GLMVec3 diff3 = getStraightProjection(cam, pointZ) - originalProjection;

        return glm::transpose(GLMMat3(diff1, diff2, diff3));
    }

    GLMVec3 BasicPhotogrammetristAccuracyModel::getStraightProjection(CameraMatrix &P, GLMVec3 &point)
    {
        GLMVec4 projection = P * GLMVec4(point, 1.0f);
        return GLMVec3(projection.x, projection.y, projection.z);
    }



} // namespace meshac
