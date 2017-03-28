#ifndef MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
#define MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H


#include <meshac/alias_definition.hpp>
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {
    
    class ComputerVisionAccuracyModel : public PhotogrammetristAccuracyModel {
    public:

        ComputerVisionAccuracyModel(ImageFileMap &fileMap, CameraMatrixList &cameras, 
                                GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam);
        
        ComputerVisionAccuracyModel(ImageFileMap &fileMap, CameraList &cameras, 
                                GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam);

        ComputerVisionAccuracyModel(SfMData &data);
        
        ~ComputerVisionAccuracyModel() { };

    protected:
        /*
         * 
         */
        virtual EigVector4 evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);   
        
    };


} // namespace meshac


#endif // MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
