#ifndef MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/CrossRatioTuple.hpp>
#include <meshac/CRTuplesGenerator.hpp>

namespace meshac {

    class ImagePointVarianceEstimator {
    public:
        ImagePointVarianceEstimator(StringList &fileList, GLMVec2ArrayList &camObservations, DoublePair &pixelSize);
        virtual ~ImagePointVarianceEstimator();

        /*
         * Computes the variance for a 2D point in the given camera.
         */
        virtual CrossRatioTupleSetVariance estimateVarianceMatrixForPoint(GLMVec2 &point, int camIndex);
        

        /*
         * Setter and getter for camera's observations.
         */
        void setCameraObservations(GLMVec2ArrayList &camObservations);
        void setCameraObservations(GLMVec2ArrayList &camObservations, IntList &camIndexs);
        void updateCameraObservations(GLMVec2ArrayList &camObservations, IntList &indexs);
        GLMVec2ArrayList getCameraObeservations();

    protected:
        /*
         * Computes the variances for each CrossRatio tuple containing a given 2D point.
         */
        virtual EigMatrixList collectPointVarianceMatrix(CrossRatioTupleSet &imageTupleSet, GLMVec2 &point);

        /*
         * Getter and setter for the variances of a given cameraId. 
         */
        double getVarianceSet(int camIndex);
        void setVarianceSet(double variance, int camIndex);

        /*
         * Computes the standard deviation for the given tuple. 
         */
        virtual EigMatrix estimateSTDForTuple(CrossRatioTuple &tuple, CrossRatioTupleSet &tupleSet);

        /*
         * Computes the standard deviation for the set of tuples.
         */
        virtual double estimateSTDTupleSet(CrossRatioTupleSet &tupleSet); // variance of CR is 1/ (N-1) * sum of (cr_i - cr_avg)^2

    private:

        void buildPixelSizeMatrix();
        CrossRatioTupleSet removeTuples(CrossRatioTupleSet &imageTupleSet, GLMVec2 &point);

        //ListCrossRatioTupleSet listTupleSet;

        CRTuplesGeneratorPtr tuplesGenerator;

        DoubleList variances;
        DoublePair pixelSize;
        EigMatrix pixelSizeDiagonalMatrix;

    };

    typedef ImagePointVarianceEstimator * ImagePointVarianceEstimatorPtr;

} // namespace meshac

#endif // MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
