
#ifndef MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H

#include <glm/gtc/epsilon.hpp>

#include <meshac/alias_definition.hpp>
#include <meshac/AccuracyModel.hpp>
#include <meshac/meshac_type_definition.hpp>


namespace meshac {

    class Point3DVarianceEstimator {
    public:
        Point3DVarianceEstimator(AccuracyModelPtr accuracyModel, GLMListVec3 &points);
        ~Point3DVarianceEstimator();

        virtual EigMatrix getVariaceMatrixForPoint(GLMVec3 &point);
        virtual EigMatrix getVariaceMatrixForPoint(int pointIndex);

        /*
         * Computes the uncertainity measure for the given point.
         */
        virtual double computeVarianceForPoint(GLMVec3 &point);
        virtual double computeVarianceForPoint(int pointIndex);

        /*
         * Setter and getter for the accuracy model used.
         */
        void setAccuracyModel(AccuracyModelPtr accuracyModel);
        AccuracyModelPtr getAccuracyModel();

        /*
         * Setter and getter for the 3D points.
         */
        GLMListVec3 get3DPoints();
        void set3DPoints(GLMListVec3 &points);
        void append3DPoints(GLMListVec3 &points);
        void append3DPoints(GLMVec3 &points);

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat) = 0; 
        /*
         * Abstract method to compute the variance given the matrix.
         */
        virtual double computeVarianceFromMatrix(EigMatrix &varianceMatrix) = 0;

    private:
        AccuracyModelPtr accuracyModel;
        GLMListVec3 points;

        const GLMVec3 EPSILON = GLMVec3(SENSIBILITY);

    };

    typedef Point3DVarianceEstimator* Point3DVarianceEstimatorPtr;
    
} // namespace meshac


#endif // MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
