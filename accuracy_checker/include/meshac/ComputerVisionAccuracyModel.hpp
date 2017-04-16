#ifndef MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
#define MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H

#include <realtimeMR/SfMData.h>

#include <meshac/alias_definition.hpp>
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {
    
    class ComputerVisionAccuracyModel : public PhotogrammetristAccuracyModel {
    public:

        ComputerVisionAccuracyModel(StringList &fileList, CameraMatrixList &cameras, GLMListArrayVec2 &camObservations,
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);
        
        ComputerVisionAccuracyModel(StringList &fileList, CameraList &cameras, GLMListArrayVec2 &camObservations,
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);

        ComputerVisionAccuracyModel(SfMData &data, DoublePair &pixelSize);
        ComputerVisionAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize);
        
        ~ComputerVisionAccuracyModel() { };

    protected:
        /*
         * Evaluates the function using homogeneous coordinates.
         */
        virtual EigVector evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);   
        
    };


} // namespace meshac


#endif // MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
