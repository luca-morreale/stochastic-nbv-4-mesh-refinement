/*
 * SfMData.h
 *
 *  Created on: 16 mar 2016
 *      Author: Andrea Romanoni
 */

#ifndef EVALUATION_CAMERA_POSITION_SFM_DATA_
#define EVALUATION_CAMERA_POSITION_SFM_DATA_

#include <aliases.h>
#include <type_definition.h>

namespace cameval {

    struct SfMData {

        int numPoints_;
        int numCameras_;

        GLMVec3List points_;
        CameraTypeList camerasList_;
        StringList camerasPaths_;

        IntArrayList camViewingPointN_;
        IntArrayList pointsVisibleFromCamN_;
        GLMVec2ArrayList point2DoncamViewingPoint_;

        GLMVec2ArrayList camViewing2DPoint_;
        MapIntGLMVe2List point3DTo2DThroughCam_;
        
        int imageWidth_, imageHeight_;
    };

} // namespace cameval

#endif /* EVALUATION_CAMERA_POSITION_SFM_DATA_ */
