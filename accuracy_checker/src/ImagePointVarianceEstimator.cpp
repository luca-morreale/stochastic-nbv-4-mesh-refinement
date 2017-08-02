#include <meshac/ImagePointVarianceEstimator.hpp>

namespace meshac {


    ImagePointVarianceEstimator::ImagePointVarianceEstimator(StringList &fileList, GLMVec2ArrayList &camObservations, DoublePair &pixelSize)
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

    void ImagePointVarianceEstimator::setCameraObservations(GLMVec2ArrayList &camObservations)
    {
        this->tuplesGenerator->setCamObservations(camObservations);
        this->variances.assign(camObservations.size(), -1);
    }

    void ImagePointVarianceEstimator::setCameraObservations(GLMVec2ArrayList &camObservations, IntList &camIndexs)
    {
        for (int i = 0; i < camObservations.size(); i++) {
            this->tuplesGenerator->setCamObservations(camObservations[i], i);
        }
        this->variances.assign(camObservations.size(), -1);
    }

    void ImagePointVarianceEstimator::updateCameraObservations(GLMVec2ArrayList &camObservations, IntList &indexs)
    {
        this->tuplesGenerator->updateCamObservations(camObservations, indexs);
        for (auto obsInd : indexs) {
            this->variances[obsInd] = -1;
        }
    }

    GLMVec2ArrayList ImagePointVarianceEstimator::getCameraObeservations()
    {
        return this->tuplesGenerator->getCamObservations();
    }


    CrossRatioTupleSetVariance ImagePointVarianceEstimator::estimateVarianceMatrixForPoint(GLMVec2 &point, int camIndex)
    {
        DoubleList covariancePointMatrix;
        CrossRatioTupleSet imageTupleSet = this->tuplesGenerator->determineTupleOfFourPointsForCam(camIndex);
        imageTupleSet = removeTuples(imageTupleSet, point);

        
        EigMatrixList variances = collectPointVarianceMatrix(imageTupleSet, point);
        
        for (int i = 0; i < variances.size(); i++) {
            appendMatrixDiagonalToVector(variances[i], covariancePointMatrix);
        }

        EigMatrix variance = generateDiagonalMatrix(covariancePointMatrix);
        return std::make_pair(imageTupleSet, variance);
    }

    CrossRatioTupleSet ImagePointVarianceEstimator::removeTuples(CrossRatioTupleSet &imageTupleSet, GLMVec2 &point)
    {
        CrossRatioTupleSet tmp;
        
        #pragma omp parallel
        for(auto it = imageTupleSet.begin(); it != imageTupleSet.end(); ++it) {
            if (it->isInTuple(point)) {
                #pragma omp critical
                tmp.insert(*it);
            }
        }
        
        return tmp;
    }

    EigMatrixList ImagePointVarianceEstimator::collectPointVarianceMatrix(CrossRatioTupleSet &imageTupleSet, GLMVec2 &point)
    {
        EigMatrixList variances;

        for (CrossRatioTuple tuple : imageTupleSet) {
            if (tuple.isInTuple(point)) {
                EigMatrix mat = this->estimateSTDForTuple(tuple, imageTupleSet);
                variances.push_back(mat);
            }
        }

        return (variances.size() > 0) ? variances : EigMatrixList(1, EigZeros(2));
    }

    EigMatrix ImagePointVarianceEstimator::estimateSTDForTuple(CrossRatioTuple &tuple, CrossRatioTupleSet &tupleSet)    // should be multiplied for size of pize!
    {
        double varianceSet = this->estimateSTDTupleSet(tupleSet);

        auto jacobian = tuple.jacobian();   // row vector 1x4

        double lambda = tuple.avgDistance();

        double jacobianNorm = jacobian.norm();

        double varianceTuple = varianceSet * lambda / (jacobianNorm * jacobianNorm);

        return varianceTuple * pixelSizeDiagonalMatrix;      // size of pixel 0.3527 mm x 0.3527 mm
    }


    double ImagePointVarianceEstimator::estimateSTDTupleSet(CrossRatioTupleSet &tupleSet)
    {
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

        return summation.sum() / (N - 1);
    }

    /*
     * Getter and setter for cross ratio's variance of a single tuple.
     */
    double ImagePointVarianceEstimator::getVarianceSet(int camIndex)
    {
        if (this->variances[camIndex] < 0) {
            CrossRatioTupleSet imageTupleSet = this->tuplesGenerator->determineTupleOfFourPointsForCam(camIndex);
            this->estimateSTDTupleSet(imageTupleSet);
        }
        return this->variances[camIndex];
    }

    void ImagePointVarianceEstimator::setVarianceSet(double variance, int camIndex)
    {
        this->variances[camIndex] = variance;
    }


} // namespace meshac
