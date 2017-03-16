
#ifndef MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
#define MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H

#include <alias_definition.hpp>
#include <CrossRatioTuple.hpp>

namespace meshac {

    class ImagePointVarianceEstimator {
    public:
        ImagePointVarianceEstimator(ListCrossRatioTupleSet listTupleSet);
        ~ImagePointVarianceEstimator();


        virtual EigMatrix4 estimateVarianceForTuple(ListCrossRatioTupleSet listTupleSet, CrossRatioTuple &tuple, int setIndex);
        virtual EigMatrix4 estimateVarianceForTuple(CrossRatioTuple &tuple, int setIndex);
        
        virtual EigMatrix4 estimateVarianceForPoint(ListCrossRatioTupleSet listTupleSet, GLMVec2 &point, int setIndex);
        virtual EigMatrix4 estimateVarianceForPoint(GLMVec2 &point, int setIndex);
        

        ListCrossRatioTupleSet getCrossRatioTupleSetList();
        void setCrossRatioTupleSetList(ListCrossRatioTupleSet listTupleSet);

        void updateCrossRatioTupleSet(CrossRatioTupleSet tupleSet, int indexSet);


    protected:
        double getVarianceSet(int setIndex);
        void setVarianceSet(double variance, int setIndex);

    
        double estimateVarianceTupleSet(int indexSet); // variance of CR is 1/ (N-1) * sum of (cr_i - cr_avg)^2

    private:
        ListCrossRatioTupleSet listTupleSet;

        DoubleList variances;

    };

    typedef ImagePointVarianceEstimator * ImagePointVarianceEstimatorPtr;

} // namespace meshac

#endif // MESH_ACCURACY_IMAGE_POINT_VARIANCE_ESTIMATOR_H
