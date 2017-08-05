#ifndef MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
#define MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

#include <realtimeMR/SfMData.h>

#include <meshac/ResidualPointAccuracyModel.hpp>
#include <meshac/alias_definition.hpp>
#include <meshac/ImagePointVarianceEstimator.hpp>
#include <meshac/InvalidUpdateException.hpp>

namespace meshac {
    
    class PhotogrammetristAccuracyModel : public ResidualPointAccuracyModel {
    public:
        PhotogrammetristAccuracyModel(SfMData &data, DoublePair &pixelSize);
        PhotogrammetristAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize);
        
        ~PhotogrammetristAccuracyModel();
        
        /*
         * Computes the matrix representing the uncertainty of the 3D point.
         * It takes into account all the observation of that point.
         */
        virtual EigMatrixList getAccuracyForPointByImage(int index3DPoint);

        /*
         * Getter and setter of all the private variables.
         */
        ImagePointVarianceEstimatorPtr getVarianceEstimator();

        virtual void setCameraObservations(GLMVec2ArrayList &newCamObservations);
        virtual void setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        virtual void updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        

    protected:
        /*
         * Initializes all members.
         */
        virtual void initMembers(DoublePair &pixelSize);

        /*
         * Puts at the beginning of each file path stored the given prefix.
         */
        virtual void fixImagesPath(std::string &pathPrefix);

        /*
         * Evaluates the photogrammetrist's function in the given point with the given camera.
         * This function does not use homogeneous coordinates.
         */
        virtual EigVector evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);

        /*
         * Computes the jacobian of the function.
         */
        virtual EigMatrix computeJacobian(CrossRatioTuple &tuple, CameraMatrix &cam);
        virtual EigVector computeSingleJacobianFor(EigVector &original, CameraMatrix &cam, GLMVec2 &pointH);

        /*
         * For each value in the pointMatrixList computes the covariance matrix and append the result in destList.
         */
        virtual void iterativeEstimationOfCovariance(EigMatrixList &destList, EigMatrixList &pointMatrixList, EigMatrix &jacobian);

        /*
         * 
         */
        virtual void updateVariancesList(DoubleList &varianesList, EigMatrix &varianceMat, EigMatrixList &jacobianList, EigMatrix &jacobianMat);
        
        virtual EigMatrix getAccuracyForPointInImage(CamPointPair &cameraObsPair);

        virtual EigMatrix replicateVarianceForTuple(EigMatrix &singleVariance);

    private:
        EigMatrixList computesProducts(EigMatrixList &jacobian, EigMatrixList &pointCovariance);

        ImagePointVarianceEstimatorPtr varianceEstimator;

        typedef ResidualPointAccuracyModel super;
    };


} // namespace meshac


#endif // MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
