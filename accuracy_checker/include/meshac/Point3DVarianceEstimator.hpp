
#ifndef MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H

#include <glm/gtc/epsilon.hpp>

#include <meshac/alias_definition.hpp>
#include <meshac/PointAccuracyModel.hpp>
#include <meshac/type_definition.hpp>


namespace meshac {

    class Point3DVarianceEstimator {
    public:
        Point3DVarianceEstimator(PointAccuracyModelPtr accuracyModel, GLMVec3List &points);
        virtual ~Point3DVarianceEstimator();

        virtual EigMatrix computeVariaceMatrixForPoint(GLMVec3 &point);
        virtual EigMatrix computeVariaceMatrixForPoint(int pointIndex);

        /*
         * Computes the uncertainity measure for the given point.
         */
        virtual double computeSingleVarianceForPoint(GLMVec3 &point);
        virtual double computeSingleVarianceForPoint(int pointIndex);
        /*
         * Setter and getter for the accuracy model used.
         */
        void setAccuracyModel(PointAccuracyModelPtr accuracyModel);
        PointAccuracyModelPtr getAccuracyModel();

        /*
         * Setter and getter for the 3D points.
         */
        GLMVec3List get3DPoints();
        void set3DPoints(GLMVec3List &points);
        void append3DPoints(GLMVec3List &points);
        void append3DPoints(GLMVec3 &points);

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat) = 0; 
        /*
         * Abstract method to compute the variance given the matrix.
         */
        virtual double computeVarianceFromMatrix(EigMatrix &varianceMatrix) = 0;

    private:
        PointAccuracyModelPtr accuracyModel;
        GLMVec3List points;

        const GLMVec3 EPSILON = GLMVec3(SENSIBILITY);

    };

    typedef Point3DVarianceEstimator* Point3DVarianceEstimatorPtr;
    
} // namespace meshac


#endif // MESH_ACCURACY_POINT_3D_VARIANCE_ESTIMATOR_H
