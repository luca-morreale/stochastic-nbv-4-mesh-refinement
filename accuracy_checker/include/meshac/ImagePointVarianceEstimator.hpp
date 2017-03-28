#ifndef MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H

#include <meshac/alias_definition.hpp>
#include <meshac/CrossRatioTuple.hpp>
#include <meshac/CRTuplesGenerator.hpp>

namespace meshac {

    class ImagePointVarianceEstimator {
    public:
        ImagePointVarianceEstimator(ImageFileMap &fileMap, GLMListArrayVec2 &camObservations);
        ~ImagePointVarianceEstimator();

        void setCameraObservations(GLMListArrayVec2 &camObservations);
        void setCameraObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs);
        void updateCameraObservations(GLMListArrayVec2 &camObservations, IntList &indexs);
        GLMListArrayVec2 getCameraObeservations();

        virtual EigMatrix estimateVarianceForPoint(GLMVec2 &point, int camIndex);



    protected:
        double getVarianceSet(int setIndex);
        void setVarianceSet(double variance, int setIndex);

        virtual EigMatrix estimateVarianceForTuple(CrossRatioTuple &tuple, int setIndex);

        virtual double estimateVarianceTupleSet(int indexSet); // variance of CR is 1/ (N-1) * sum of (cr_i - cr_avg)^2

    private:
        //ListCrossRatioTupleSet listTupleSet;

        CRTuplesGeneratorPtr tuplesGenerator;

        DoubleList variances;

    };

    typedef ImagePointVarianceEstimator * ImagePointVarianceEstimatorPtr;

} // namespace meshac

#endif // MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
