#ifndef MESH_ACCURACY_RESIDUAL_ACCURACY_MODEL_H
#define MESH_ACCURACY_RESIDUAL_ACCURACY_MODEL_H

#include <meshac/camera_utilities.hpp>
#include <meshac/InvalidUpdateException.hpp>
#include <meshac/PointAccuracyModel.hpp>
#include <meshac/SfMData.h>


namespace meshac {
    
    class ResidualPointAccuracyModel : public PointAccuracyModel {
    public:
        ResidualPointAccuracyModel(SfMData &data);
        ~ResidualPointAccuracyModel();
        /*
         * Computes the matrix that represents the accuracy of the 3D point.
         * A matrix for each image is returned.
         */
        virtual EigMatrixList getAccuracyForPointByImage(int index3DPoint);

        /*
         * Getter and setter of all the private variables.
         */
        CameraList getCameras();
        CameraMatrixList getCamerasMatrix();
        GLMVec2ArrayList getCamObservations();
        ListMappingGLMVec2 getMapping3DTo2DThroughCam();
        StringList getFileList();

        void setFileList(StringList &fileList);
        virtual void setCameras(CameraList &cameras);
        virtual void appendCamera(CameraType &cam);
        virtual void setCameraObservations(GLMVec2ArrayList &newCamObservations);
        virtual void setCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        virtual void updateCameraObservations(GLMVec2ArrayList &newCamObservations, IntList &camIndexs);
        virtual void updateMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints);
        virtual void setMapping3DTo2DThroughCam(ListMappingGLMVec2 &indexCams, IntList &index3DPoints);


    protected:

        virtual EigMatrix computeResidual(CamPointPair &camToPoint, GLMVec3 &point);

        CameraMatrix getCameraMatrix(int camIndex);

        virtual void camObservationGeneralUpdate(IntList &indexs, GLMVec2ArrayList &list, GLMVec2ArrayList &targetList, std::string errorMsg);
        virtual void mappingGeneralUpdate(IntList &indexs, ListMappingGLMVec2 &list, ListMappingGLMVec2 &targetList);

    private:
        StringList fileList;
        CameraList cameras;
        GLMVec2ArrayList camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;
        
    };

    typedef ResidualPointAccuracyModel * ResidualPointAccuracyModelPtr;

} // namespace meshac


#endif // MESH_ACCURACY_RESIDUAL_ACCURACY_MODEL_H
