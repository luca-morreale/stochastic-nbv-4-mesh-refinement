
#include <meshac/ImagePointVarianceEstimator.hpp>

namespace meshac {


    ImagePointVarianceEstimator::ImagePointVarianceEstimator(StringList &fileList, GLMListArrayVec2 &camObservations)
    {
        this->tuplesGenerator = new CRTuplesGenerator(fileList, camObservations);
        this->variances.assign(camObservations.size(), -1);
    }

    ImagePointVarianceEstimator::~ImagePointVarianceEstimator()
    {
        delete this->tuplesGenerator;

        this->variances.clear();
    }

    void ImagePointVarianceEstimator::setCameraObservations(GLMListArrayVec2 &camObservations)
    {
        this->tuplesGenerator->setCamObservations(camObservations);
        this->variances.assign(camObservations.size(), -1);
    }

    void ImagePointVarianceEstimator::setCameraObservations(GLMListArrayVec2 &camObservations, IntList &camIndexs)
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->tuplesGenerator->setCamObservations(camObservations[i], i);
        }
        this->variances.assign(camObservations.size(), -1);
    }

    void ImagePointVarianceEstimator::updateCameraObservations(GLMListArrayVec2 &camObservations, IntList &indexs)
    {
        this->tuplesGenerator->updateCamObservations(camObservations, indexs);
        for (auto obsInd : indexs) {
            this->variances[obsInd] = -1;
        }
    }

    GLMListArrayVec2 ImagePointVarianceEstimator::getCameraObeservations()
    {
        return this->tuplesGenerator->getCamObservations();
    }


    EigMatrix ImagePointVarianceEstimator::estimateVarianceMatrixForPoint(GLMVec2 &point, int camIndex)
    {
        DoubleList covariancePointMatrix;
        CrossRatioTupleSet completeTupleSet = this->tuplesGenerator->getComputedTuplesForCam(camIndex);
        
        for (CrossRatioTuple tuple : completeTupleSet) {
            if (tuple.isInTuple(point)) {
                EigMatrix mat = this->estimateSTDForTuple(tuple, camIndex);
                appendMatrixDiagonalToVector(mat, covariancePointMatrix);
            }
        }
        
        return generateDiagonalMatrix(covariancePointMatrix);
    }


    EigMatrix ImagePointVarianceEstimator::estimateSTDForTuple(CrossRatioTuple &tuple, int camIndex)
    {
        double varianceSet = this->getVarianceSet(camIndex);
        
        auto jacobian = tuple.jacobian();   // row vector 1x4
        
        double lambda = tuple.avgDistance();
        
        double jacobianNorm = jacobian.norm();
        
        double varianceTuple = varianceSet * lambda / (jacobianNorm * jacobianNorm);
        
        return varianceTuple * EigIdentity(2);
    }


    double ImagePointVarianceEstimator::estimateSTDTupleSet(int camIndex)
    {
        CrossRatioTupleSet tupleSet = this->tuplesGenerator->getComputedTuplesForCam(camIndex);
        CrossRatioTupleSet::iterator tuple = tupleSet.begin();
        int N = tupleSet.size();

        if (N == 1) {
            return 0;
        }

        double avg = 0;
        EigVector crossRatioValues(N);
        
        for (int i = 0; i < N; i++) {
            crossRatioValues[i] = (*tuple).crossRatio();
            avg += crossRatioValues[i];

            std::advance(tuple, 1);
        }
        avg /= N;
        EigVector summation = crossRatioValues - avg * EigVector::Ones(N);
        summation = summation.array().square();

        this->setVarianceSet(summation.sum() / (N - 1), camIndex);
        return this->getVarianceSet(camIndex);
    }

    /*
     * Getter and setter for cross ratio's variance of a single tuple.
     */
    double ImagePointVarianceEstimator::getVarianceSet(int camIndex)
    {
        if (this->variances[camIndex] < 0) {
            this->estimateSTDTupleSet(camIndex);
        }
        return this->variances[camIndex];
    }

    void ImagePointVarianceEstimator::setVarianceSet(double variance, int camIndex)
    {
        this->variances[camIndex] = variance;
    }


} // namespace meshac
