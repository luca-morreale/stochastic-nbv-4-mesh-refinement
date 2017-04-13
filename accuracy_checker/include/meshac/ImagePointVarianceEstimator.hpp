#ifndef MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/CrossRatioTuple.hpp>
#include <meshac/CRTuplesGenerator.hpp>

namespace meshac {

    class ImagePointVarianceEstimator {
    public:
        ImagePointVarianceEstimator(StringList &fileList, GLMListArrayVec2 &camObservations);
        ~ImagePointVarianceEstimator();

        /*
         * Computes the variance for a 2D point in the given camera.
         */
        virtual EigMatrix estimateVarianceMatrixForPoint(GLMVec2 &point, int camIndex);

        /*
         * Setter and getter for camera's observations.
         */
        void setCameraObservations(GLMListArrayVec2 &camObservations);
        void setCameraObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs);
        void updateCameraObservations(GLMListArrayVec2 &camObservations, IntList &indexs);
        GLMListArrayVec2 getCameraObeservations();

    protected:
        /*
         * Getter and setter for the variances of a given cameraId. 
         */
        double getVarianceSet(int camIndex);
        void setVarianceSet(double variance, int camIndex);

        /*
         * Computes the standard deviation for the given tuple. 
         */
        virtual EigMatrix estimateSTDForTuple(CrossRatioTuple &tuple, int setIndex);

        /*
         * Computes the standard deviation for the set of tuples.
         */
        virtual double estimateSTDTupleSet(int indexSet); // variance of CR is 1/ (N-1) * sum of (cr_i - cr_avg)^2

    private:
        //ListCrossRatioTupleSet listTupleSet;

        CRTuplesGeneratorPtr tuplesGenerator;

        DoubleList variances;

    };

    typedef ImagePointVarianceEstimator * ImagePointVarianceEstimatorPtr;

} // namespace meshac

#endif // MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
