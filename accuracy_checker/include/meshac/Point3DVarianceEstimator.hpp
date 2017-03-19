
#ifndef MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H

#include <glm/gtc/epsilon.hpp>

#include <meshac/alias_definition.hpp>
#include <meshac/AccuracyModel.hpp>
#include <meshac/meshac_type_definition.hpp>


namespace meshac {

    class Point3DVarianceEstimator {
    public:
        Point3DVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 points);
        ~Point3DVarianceEstimator();

        virtual EigMatrix computeVariaceForPoint(GLMVec3 point);

        void setAccuracyModel(AccuracyModelPtr accuracyModel);
        AccuracyModelPtr getAccuracyModel();

        GLMListVec3 get3DPoints();
        void set3DPoints(GLMListVec3 points);
        void append3DPoints(GLMListVec3 points);
        void append3DPoints(GLMVec3 points);

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat) = 0;

    private:
        AccuracyModelPtr accuracyModel;
        GLMListVec3 points;

        const GLMVec3 EPSILON = GLMVec3(SENSIBILITY);

    };
    
} // namespace meshac


#endif // MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
