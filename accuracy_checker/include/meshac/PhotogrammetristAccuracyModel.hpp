
#ifndef MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
#define MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

#include <boost/bind.hpp>

#include <meshac/AccuracyModel.hpp>
#include <meshac/alias_definition.hpp>
#include <meshac/ImagePointVarianceEstimator.hpp>
#include <meshac/InvalidUpdateException.hpp>

namespace meshac {
    
    class PhotogrammetristAccuracyModel : public AccuracyModel {
    public:
        PhotogrammetristAccuracyModel(GLMListVec3 points3D, CameraMatrixList cameras, 
                        GLMListArrayVec2 camObservations, ListMappingGLMVec2 point3DTo2DThroughCam, int obsWidth, int obsHeight);
        
        PhotogrammetristAccuracyModel(GLMListVec3 points3D, CameraList cameras, 
                        GLMListArrayVec2 camObservations, ListMappingGLMVec2 point3DTo2DThroughCam, int obsWidth, int obsHeight);
        
        PhotogrammetristAccuracyModel(SfMData data);
        
        ~PhotogrammetristAccuracyModel();
        
        
        /*
         * Computes the matrix of uncertainty for the specified point in coordinates 3D
         * ARGS:
         * int index3DPoint     index of the 3D point wrt to those int the list of 3D points given.
         *
         * RETURNS:
         * a 3x3 matrix for each correspongind 2D point representative of the uncertainty.
         */
        virtual EigMatrixList getAccuracyForPoint(int index3DPoint);


        CameraMatrixList getCamerasMatrix();
        GLMListArrayVec2 getCamObservations();
        ListMappingGLMVec2 getMapping3DTo2DThroughCam();
        std::pair<int, int> getObservationSize();

        void appendCamera(CameraMatrix cam);
        void setCameraObservations(GLMListArrayVec2 newCamObservations);
        void setCameraObservations(GLMListArrayVec2 newCamObservations, IntList camIndexs);
        void updateCameraObservations(GLMListArrayVec2 newCamObservations, IntList camIndexs);
        void updateMapping3DTo2DThroughCam(ListMappingGLMVec2 indexCams, IntList index3DPoints);
        void setMapping3DTo2DThroughCam(ListMappingGLMVec2 indexCams, IntList index3DPoints);


    protected:
        /*
         * Initializes all members.
         */
        virtual void initMembers();

        /*
         * Evaluates the photogrammetrist's function in the given point with the given camera.
         */
        virtual EigVector4 evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);

        /*
         * 
         * 
         */
        virtual EigMatrix computeJacobian(CameraMatrix &cam, GLMVec2 &point);




        virtual void camObservationGeneralUpdate(IntList &indexs, GLMListArrayVec2 &list, GLMListArrayVec2 &targetList, std::string errorMsg);
        virtual void mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList);


        /*
         * 
         */
        void setCameras(CameraMatrixList cameras);
        void setCamObservations(GLMListArrayVec2 camObservations);
        void setVisibilityOfPoints(ListMappingGLMVec2 point3DTo2DThroughCam);

        float getXh();
        float getYh();

    private:
        CameraMatrixList extractCameraMatrix(CameraList &cameras);

        ImagePointVarianceEstimatorPtr varianceEstimator;


        CameraMatrixList cameras;
        GLMListArrayVec2 camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;

        int obsWidth;
        int obsHeight;

        const float xh = 0.01;
        const float yh = 0.01;

    };


} // namespace meshac


#endif // MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

