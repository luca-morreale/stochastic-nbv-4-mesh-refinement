/*
 * SfMData.h
 *
 *  Created on: 16 mar 2016
 *      Author: Andrea Romanoni
 */

#ifndef MESH_ACCURACY_SFM_DATA
#define MESH_ACCURACY_SFM_DATA

#include <meshac/alias_definition.hpp>
#include <meshac/type_definition.hpp>

namespace meshac {

    struct SfMData {

        int numPoints_;
        int numCameras_;

        GLMVec3List points_;
        CameraList camerasList_;
        StringList camerasPaths_;

        IntArrayList camViewingPointN_;
        IntArrayList pointsVisibleFromCamN_;
        GLMVec2ArrayList point2DoncamViewingPoint_;

        std::vector<std::vector<glm::vec2> > camViewing2DPoint_;
        std::vector<std::map<int, glm::vec2> > point3DTo2DThroughCam_;
        
        int imageWidth_, imageHeight_;
    };

} // namespace meshac

#endif /* MESH_ACCURACY_SFM_DATA */
