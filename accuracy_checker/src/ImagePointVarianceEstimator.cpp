
#include <meshac/ImagePointVarianceEstimator.hpp>

namespace meshac {


    ImagePointVarianceEstimator::ImagePointVarianceEstimator(StringList &fileList, GLMListArrayVec2 &camObservations, DoublePair &pixelSize)
    {
        this->tuplesGenerator = new CRTuplesGenerator(fileList, camObservations);
        this->variances.assign(camObservations.size(), -1);
        this->pixelSize = pixelSize;
        this->buildPixelSizeMatrix();
    }

    ImagePointVarianceEstimator::~ImagePointVarianceEstimator()
    {
        delete this->tuplesGenerator;

        this->variances.clear();
    }

    void ImagePointVarianceEstimator::buildPixelSizeMatrix()
    {
        this->pixelSizeDiagonalMatrix = EigIdentity(2);
        this->pixelSizeDiagonalMatrix(0, 0) *= this->pixelSize.first;
        this->pixelSizeDiagonalMatrix(1, 1) *= this->pixelSize.second;
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
        EigMatrixList variances = collectPointVarianceMatrix(point, camIndex);

        #pragma omp parallel for
        for (int i = 0; i < variances.size(); i++) {
            appendMatrixDiagonalToVector(variances[i], covariancePointMatrix);
        }

        return generateDiagonalMatrix(covariancePointMatrix);
    }

    EigMatrixList ImagePointVarianceEstimator::collectPointVarianceMatrix(GLMVec2 &point, int camIndex)
    {
        EigMatrixList variances;

        CrossRatioTupleSet completeTupleSet = this->tuplesGenerator->determineTupleOfFourPointsForCam(camIndex);
        std::cout << "complete tuple set " << completeTupleSet.size() << std::endl;
        for (CrossRatioTuple tuple : completeTupleSet) {
            if (tuple.isInTuple(point)) {
                EigMatrix mat = this->estimateSTDForTuple(tuple, camIndex);
                variances.push_back(mat);
            }
        }

        return (variances.size() > 0) ? variances : EigMatrixList(1, EigZeros(2));
    }


    EigMatrix ImagePointVarianceEstimator::estimateSTDForTuple(CrossRatioTuple &tuple, int camIndex)    // should be multiplied for size of pize!
    {
        double varianceSet = this->estimateSTDTupleSet(camIndex);

        auto jacobian = tuple.jacobian();   // row vector 1x4

        double lambda = tuple.avgDistance();

        double jacobianNorm = jacobian.norm();

        double varianceTuple = varianceSet * lambda / (jacobianNorm * jacobianNorm);

        return varianceTuple * pixelSizeDiagonalMatrix;      // size of pixel 0.3527 mm x 0.3527 mm
    }


    double ImagePointVarianceEstimator::estimateSTDTupleSet(int camIndex)
    {
        CrossRatioTupleSet tupleSet = this->tuplesGenerator->determineTupleOfFourPointsForCam(camIndex);
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
