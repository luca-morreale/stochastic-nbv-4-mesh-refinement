
#ifndef MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
#define MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H


#include <PhotogrammetristAccuracyModel.hpp>
#include <alias_definition.hpp>

namespace meshac {
    
    class ComputerVisionAccuracyModel : public PhotogrammetristAccuracyModel {
    public:

        ComputerVisionAccuracyModel(GLMList3DVec points3D, CameraMatrixList cameras, GLMListArray2DVec camObservations, 
                                                ListMappingGLM2DVec point3DTo2DThroughCam, int obsWidth, int obsHeight);
        
        ComputerVisionAccuracyModel(GLMList3DVec points3D, CameraList cameras, GLMListArray2DVec camObservations, 
                                                ListMappingGLM2DVec point3DTo2DThroughCam, int obsWidth, int obsHeight);

        ComputerVisionAccuracyModel(SfMData data);
        
        ~ComputerVisionAccuracyModel() { };

    protected:
        /*
         * 
         */
        virtual EigVector4 evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);   
        
    };


} // namespace meshac


#endif // MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
