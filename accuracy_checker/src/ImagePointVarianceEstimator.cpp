
#include <ImagePointVarianceEstimator.hpp>

namespace meshac {

/*
 * Constructor and Destructor.
 */
ImagePointVarianceEstimator::ImagePointVarianceEstimator() { }

ImagePointVarianceEstimator::ImagePointVarianceEstimator(CrossRatioTupleSet tupleSet)
{
    this->crTupleSet = tupleSet;
}

ImagePointVarianceEstimator::~ImagePointVarianceEstimator() { }

/*
 * Getter and setter for CrossRatioTupleSet. 
 */
CrossRatioTupleSet ImagePointVarianceEstimator::getCrossRatioTupleSet()
{
    return this->crTupleSet;
}

void ImagePointVarianceEstimator::setCrossRatioTupleSet(CrossRatioTupleSet tupleSet)
{
    this->crTupleSet = tupleSet;
}


/*
 * Getter and setter for cross ratio's variance of a single tuple.
 */
double ImagePointVarianceEstimator::getVarianceCRTuple()
{
    if (this->varianceCRTuple < 0){
        this->estimateVarianceCRTuple();
    }
    return this->varianceCRTuple;
}

void ImagePointVarianceEstimator::setVarianceCRTuple(double variance)
{
    this->varianceCRTuple = variance;
}

/*
 * Variance estimation.
 */
EigMatrix ImagePointVarianceEstimator::estimateVarianceForTuple(CrossRatioTupleSet tupleSet, CrossRatioTuple tuple)
{
    this->setCrossRatioTupleSet(tupleSet);
    return this->estimateVarianceForTuple(tuple);
}

EigMatrix ImagePointVarianceEstimator::estimateVarianceForTuple(CrossRatioTuple tuple)
{
    double varianceCR = this->getVarianceCRTuple();

    auto jacobian = tuple.jacobian();
    double lambda = tuple.avgDistance();

    double jacobianNorm = jacobian.norm();

    double varianceTuple = varianceCR * lambda / (jacobianNorm * jacobianNorm);

    return varianceTuple * EigIdentity(4);
}


EigMatrix ImagePointVarianceEstimator::estimateVarianceForTuple(CrossRatioTupleSet tupleSet, glm::vec2 point)
{
    this->setCrossRatioTupleSet(tupleSet);
    return this->estimateVarianceForPoint(point);
}

EigMatrix ImagePointVarianceEstimator::estimateVarianceForPoint(glm::vec2 point)
{
    int counterTuples = 0;

    auto variance = EigZeros(4);

    for (auto tuple : crTupleSet) {
        if (tuple.isInTuple(point)) {
            variance += estimateVarianceForTuple(tuple);
            ++counterTuples;
        }
    }

    return variance / counterTuples;
}



double ImagePointVarianceEstimator::estimateVarianceCRTuple()
{
    double avg = 0;
    int N = this->crTupleSet.size();

    EigVector crossRatioValues(N);
    auto tuple = this->crTupleSet.begin();

    for (int i = 0; i < N; i++) {
        crossRatioValues[i] = (*tuple).crossRatio();
        avg += crossRatioValues[i];

        std::advance(tuple, 1);
    }
    EigVector summation = crossRatioValues - avg * EigVector::Ones(N);

    this->setVarianceCRTuple(summation.sum() / (N - 1));
    return this->getVarianceCRTuple();
}


} // namespace meshac
