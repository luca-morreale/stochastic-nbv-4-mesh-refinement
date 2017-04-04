#ifndef MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
#define MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H

#include <manifoldReconstructor/SfMData.h>

#include <meshac/alias_definition.hpp>
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {
    
    class ComputerVisionAccuracyModel : public PhotogrammetristAccuracyModel {
    public:

        ComputerVisionAccuracyModel(StringList &fileList, CameraMatrixList &cameras, 
                                GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam);
        
        ComputerVisionAccuracyModel(StringList &fileList, CameraList &cameras, 
                                GLMListArrayVec2 &camObservations, ListMappingGLMVec2 &point3DTo2DThroughCam);

        ComputerVisionAccuracyModel(SfMData &data);
        ComputerVisionAccuracyModel(SfMData &data, std::string &pathPrefix);
        
        ~ComputerVisionAccuracyModel() { };

    protected:
        /*
         * 
         */
        virtual EigVector evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);   
        
    };


} // namespace meshac


#endif // MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
