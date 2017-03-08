
#ifndef MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H

#include <alias_definition.hpp>
#include <meshac_type_definition.hpp>

namespace meshac {

    class ImagePointVarianceEstimator {
    public:
        ImagePointVarianceEstimator();
        ImagePointVarianceEstimator(CrossRatioTupleSet tupleSet);
        ~ImagePointVarianceEstimator();

        virtual EigMatrix estimateVarianceForTuple(CrossRatioTuple tuple);
        virtual EigMatrix estimateVarianceForTuple(CrossRatioTupleSet tupleSet, CrossRatioTuple tuple);

        virtual EigMatrix estimateVarianceForPoint(glm::vec2 point);
        virtual EigMatrix estimateVarianceForTuple(CrossRatioTupleSet tupleSet, glm::vec2 point);

        CrossRatioTupleSet getCrossRatioTupleSet();
        void setCrossRatioTupleSet(CrossRatioTupleSet tupleSet);


    protected:
        double getVarianceCRTuple();
        void setVarianceCRTuple(double variance);


        double estimateVarianceCRTuple(); // variance of CR is 1/ (N-1) * sum of (cr_i - cr_avg)^2

    private:
        CrossRatioTupleSet crTupleSet;

        double varianceCRTuple = -1;


    };

} // namespace meshac

#endif // MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
