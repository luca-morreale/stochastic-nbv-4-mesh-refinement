
#include <meshac/ImagePointVarianceEstimator.hpp>

namespace meshac {


    ImagePointVarianceEstimator::ImagePointVarianceEstimator(GLMListArrayVec2 camObservations, int obsWidth, int obsHeight)
    {
        this->tuplesGenerator = new CRTuplesGenerator(camObservations, obsWidth, obsHeight);
    }

    ImagePointVarianceEstimator::~ImagePointVarianceEstimator()
    {
        delete this->tuplesGenerator; 
    }

    void ImagePointVarianceEstimator::setCameraObservations(GLMListArrayVec2 camObservations)
    {
        this->tuplesGenerator->setCamObservations(camObservations);
    }

    void ImagePointVarianceEstimator::setCameraObservations(GLMListArrayVec2 camObservations, IntList camIndexs)
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->tuplesGenerator->setCamObservations(camObservations[i], i);
        }
    }

    void ImagePointVarianceEstimator::updateCameraObservations(GLMListArrayVec2 camObservations, IntList indexs)
    {
        this->tuplesGenerator->updateCamObservations(camObservations, indexs);
    }

    GLMListArrayVec2 ImagePointVarianceEstimator::getCameraObeservations()
    {
        return this->tuplesGenerator->getCamObservations();
    }


    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForPoint(GLMVec2 &point, int camIndex)
    {
        int counterTuples = 0;
        EigMatrix4 variance = EigZeros(4);
        CrossRatioTupleSet tupleSet = this->tuplesGenerator->getComputedTuplesForCam(camIndex);

        for (CrossRatioTuple tuple : tupleSet) {
            if (tuple.isInTuple(point)) {
                variance += this->estimateVarianceForTuple(tuple, camIndex);
                ++counterTuples;
            }
        }

        return variance / counterTuples;
    }


    EigMatrix4 ImagePointVarianceEstimator::estimateVarianceForTuple(CrossRatioTuple &tuple, int camIndex)
    {
        double varianceSet = this->getVarianceSet(camIndex);

        auto jacobian = tuple.jacobian();   // row vector 1x3
        double lambda = tuple.avgDistance();

        double jacobianNorm = jacobian.norm();

        double varianceTuple = varianceSet * lambda / (jacobianNorm * jacobianNorm);

        return varianceTuple * EigIdentity(4);
    }


    double ImagePointVarianceEstimator::estimateVarianceTupleSet(int camIndex)
    {
        CrossRatioTupleSet tupleSet = this->tuplesGenerator->getComputedTuplesForCam(camIndex);
        CrossRatioTupleSet::iterator tuple = tupleSet.begin();
        int N = tupleSet.size();

        EigVector crossRatioValues(N);

        double avg = 0;
        

        for (int i = 0; i < N; i++) {
            crossRatioValues[i] = (*tuple).crossRatio();
            avg += crossRatioValues[i];

            std::advance(tuple, 1);
        }

        EigVector summation = crossRatioValues - avg * EigVector::Ones(N) / N;

        this->setVarianceSet(summation.sum() / (N - 1), camIndex);
        return this->getVarianceSet(camIndex);
    }

    /*
     * Getter and setter for cross ratio's variance of a single tuple.
     */
    double ImagePointVarianceEstimator::getVarianceSet(int camIndex)
    {
        if (this->variances[camIndex] < 0){
            this->estimateVarianceTupleSet(camIndex);
        }
        return this->variances[camIndex];
    }

    void ImagePointVarianceEstimator::setVarianceSet(double variance, int camIndex)
    {
        this->variances[camIndex] = variance;
    }


} // namespace meshac
