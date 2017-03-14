
#include <ImagePointVarianceEstimator.hpp>

namespace meshac {

    ImagePointVarianceEstimator::ImagePointVarianceEstimator(ListCrossRatioTupleSet listTupleSet)
    {
        this->listTupleSet = listTupleSet;
        this->variances.assign(listTupleSet.size(), -1.0);
    }

    ImagePointVarianceEstimator::~ImagePointVarianceEstimator()
    { /*    */ }

    /*
     * Variance estimation.
     */
    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForTuple(ListCrossRatioTupleSet listTupleSet, CrossRatioTuple &tuple, int setIndex)
    {
        this->setCrossRatioTupleSetList(listTupleSet);
        return this->estimateVarianceForTuple(tuple, setIndex);
    }

    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForTuple(CrossRatioTuple &tuple, int setIndex)
    {
        double varianceSet = this->getVarianceSet(setIndex);

        auto jacobian = tuple.jacobian();   // row vector 1x3
        double lambda = tuple.avgDistance();

        double jacobianNorm = jacobian.norm();

        double varianceTuple = varianceSet * lambda / (jacobianNorm * jacobianNorm);

        return varianceTuple * EigIdentity(4);
    }

    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForPoint(ListCrossRatioTupleSet listTupleSet, GLMVec2 &point, int setIndex)
    {
        this->setCrossRatioTupleSetList(listTupleSet);
        return this->estimateVarianceForPoint(point, setIndex);
    }

    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForPoint(GLMVec2 &point, int setIndex)
    {
        int counterTuples = 0;

        EigMatrix4 variance = EigZeros(4);

        for (CrossRatioTuple tuple : listTupleSet[setIndex]) {
            if (tuple.isInTuple(point)) {
                variance += this->estimateVarianceForTuple(tuple, setIndex);
                ++counterTuples;
            }
        }

        return variance / counterTuples;
    }


    double ImagePointVarianceEstimator::estimateVarianceTupleSet(int setIndex)
    {
        double avg = 0;
        int N = this->listTupleSet[setIndex].size();

        EigVector crossRatioValues(N);
        auto tuple = this->listTupleSet[setIndex].begin();

        for (int i = 0; i < N; i++) {
            crossRatioValues[i] = (*tuple).crossRatio();
            avg += crossRatioValues[i];

            std::advance(tuple, 1);
        }

        EigVector summation = crossRatioValues - avg * EigVector::Ones(N) / N;

        this->setVarianceSet(summation.sum() / (N - 1), setIndex);
        return this->getVarianceSet(setIndex);
    }

    /*
     * Getter and setter for ListCrossRatioTupleSet. 
     */
    ListCrossRatioTupleSet ImagePointVarianceEstimator::getCrossRatioTupleSetList()
    {
        return this->listTupleSet;
    }

    void ImagePointVarianceEstimator::setCrossRatioTupleSetList(ListCrossRatioTupleSet listTupleSet)
    {
        this->listTupleSet = listTupleSet;
        this->variances.assign(listTupleSet.size(), -1);
    }

    void ImagePointVarianceEstimator::updateCrossRatioTupleSet(CrossRatioTupleSet tupleSet, int indexSet)
    {
        this->listTupleSet[indexSet] = tupleSet;
        this->variances[indexSet] = -1;
    }

    /*
     * Getter and setter for cross ratio's variance of a single tuple.
     */
    double ImagePointVarianceEstimator::getVarianceSet(int setIndex)
    {
        if (this->variances[setIndex] < 0){
            this->estimateVarianceTupleSet(setIndex);
        }
        return this->variances[setIndex];
    }

    void ImagePointVarianceEstimator::setVarianceSet(double variance, int setIndex)
    {
        this->variances[setIndex] = variance;
    }


} // namespace meshac
