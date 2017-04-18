#ifndef MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
#define MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

#include <realtimeMR/SfMData.h>

#include <meshac/AccuracyModel.hpp>
#include <meshac/alias_definition.hpp>
#include <meshac/ImagePointVarianceEstimator.hpp>
#include <meshac/InvalidUpdateException.hpp>

namespace meshac {
    
    class PhotogrammetristAccuracyModel : public AccuracyModel {
    public:
        PhotogrammetristAccuracyModel(StringList &fileList, CameraMatrixList &cameras, GLMVec2ArrayList &camObservations,
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);
        
        PhotogrammetristAccuracyModel(StringList &fileList, CameraList &cameras, GLMVec2ArrayList &camObservations, 
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);
        
        PhotogrammetristAccuracyModel(SfMData &data, DoublePair &pixelSize);
        PhotogrammetristAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize);
        
        virtual ~PhotogrammetristAccuracyModel();
        
        
        /*
         * Computes the matrix of uncertainty for the specified point in coordinates 3D
         * ARGS:
         * int index3DPoint     index of the 3D point wrt to those int the list of 3D points given.
         *
         * RETURNS:
         * a 3x3 matrix for each correspongind 2D point representative of the uncertainty.
         */
        virtual EigMatrixList getAccuracyForPoint(int index3DPoint);

        /*
         * Computes the matrix representing the uncertainty of the 3D point.
         * It takes into account all the observation of that point.
         */
        virtual EigMatrix getCompleteAccuracyForPoint(int index3DPoint);

        /*
         * Getter and setter of all the private variables.
         */
        CameraMatrixList getCameras();
        CameraMatrixList getCamerasMatrix();
        GLMVec2ArrayList getCamObservations();
        ImagePointVarianceEstimatorPtr getVarianceEstimator();
        ListMappingGLMVec2 getMapping3DTo2DThroughCam();
        StringList getFileList();

        void setCameras(CameraMatrixList &cameras);
        void setCameras(CameraList &cameras);
        void appendCamera(CameraMatrix &cam);
        void setCameraObservations(GLMVec2ArrayList &newCamObservations);
        void setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        void updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        void updateMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints);
        void setMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints);

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
        virtual EigMatrix computeJacobian(CameraMatrix &cam, GLMVec2 &point);
        virtual EigVector computeSingleJacobianFor(EigVector &original, CameraMatrix &cam, GLMVec2 &pointH);

        /*
         * For each value in the pointMatrixList computes the covariance matrix and append the result in destList.
         */
        virtual void iterativeEstimationOfCovariance(EigMatrixList &destList, EigMatrixList &pointMatrixList, EigMatrix &jacobian);

        /*
         * 
         */
        virtual void updateVariancesList(DoubleList &varianesList, EigMatrix &varianceMat, EigMatrixList &jacobianList, EigMatrix &jacobianMat);

        /*
         * Generalized method to update the lists.
         */
        virtual void camObservationGeneralUpdate(IntList &indexs, GLMVec2ArrayList &list, GLMVec2ArrayList &targetList, std::string errorMsg);
        virtual void mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList);

        

    private:
        CameraMatrixList extractCameraMatrix(CameraList &cameras);

        ImagePointVarianceEstimatorPtr varianceEstimator;

        StringList fileList;
        CameraMatrixList cameras;
        GLMVec2ArrayList camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;

    };


} // namespace meshac


#endif // MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
