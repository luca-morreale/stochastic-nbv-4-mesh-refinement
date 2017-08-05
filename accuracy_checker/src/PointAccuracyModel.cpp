#include <meshac/PointAccuracyModel.hpp>

namespace meshac {

    PointAccuracyModel::PointAccuracyModel(GLMVec3List &points)
    {
        this->points = points;
    }

    PointAccuracyModel::~PointAccuracyModel()
    {
        this->points.clear();
    }

    EigMatrixList PointAccuracyModel::getAccuracyForPointByImage(GLMVec3 &point)
    {
        return getAccuracyForPointByImage(retreiveIndex(point));
    }

    int PointAccuracyModel::retreiveIndex(GLMVec3 &point)
    {
        for (int i = 0; i < points.size(); i++) {
            if (glm::all(glm::equal(points[i], point))) {
                return i;
            }
        }
        throw UnexpectedPointException(point);
    }

    GLMVec3List PointAccuracyModel::getPoints()
    {
        return this->points;
    }

} // namespace
