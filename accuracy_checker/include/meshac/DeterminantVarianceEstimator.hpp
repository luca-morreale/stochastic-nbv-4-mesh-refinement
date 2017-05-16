#ifndef MESH_ACCURACY_DETERMINANT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_DETERMINANT_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>


namespace meshac {
    

    class DeterminantVarianceEstimator : public Point3DVarianceEstimator {
    public:
        DeterminantVarianceEstimator(PointAccuracyModelPtr accuracyModel, GLMVec3List &points);
        virtual ~DeterminantVarianceEstimator();

    protected:
        virtual EigMatrix selectVarianceMatrix(EigMatrixList &mat); 
        /*
         * Computes the variance of the point as determinat of the given matrix.
         */
        virtual double computeVarianceFromMatrix(EigMatrix &varianceMatrix);

    };

} // namespace meshac

#endif // MESH_ACCURACY_DETERMINANT_VARIANCE_ESTIMATOR_H
