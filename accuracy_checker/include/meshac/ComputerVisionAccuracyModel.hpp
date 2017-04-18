#ifndef MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
#define MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H

#include <realtimeMR/SfMData.h>

#include <meshac/alias_definition.hpp>
#include <meshac/PhotogrammetristAccuracyModel.hpp>

namespace meshac {
    
    class ComputerVisionAccuracyModel : public PhotogrammetristAccuracyModel {
    public:

        ComputerVisionAccuracyModel(StringList &fileList, CameraMatrixList &cameras, GLMVec2ArrayList &camObservations,
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);
        
        ComputerVisionAccuracyModel(StringList &fileList, CameraList &cameras, GLMVec2ArrayList &camObservations,
                                        ListMappingGLMVec2 &point3DTo2DThroughCam, DoublePair &pixelSize);

        ComputerVisionAccuracyModel(SfMData &data, DoublePair &pixelSize);
        ComputerVisionAccuracyModel(SfMData &data, std::string &pathPrefix, DoublePair &pixelSize);
        
        ~ComputerVisionAccuracyModel() { };

        virtual EigMatrixList getAccuracyForPoint(int index3DPoint);

    protected:
        /*
         * Evaluates the function using homogeneous coordinates.
         */
        virtual EigVector evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);

        virtual EigMatrix computeJacobian(CameraMatrix &cam, GLMVec2 &point);
        
    };


} // namespace meshac


#endif // MESH_ACCURACY_COMPUTER_VISION_ACCURACY_MODEL_H
