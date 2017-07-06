#ifndef EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_
#define EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_

#include <aliases.h>

namespace cameval {


    typedef struct CameraPose {

        int index;
        EigVector3 center;
        EigMatrix3 rotation;

        CameraPose(int index, EigVector3 &center, EigMatrix3 &rot) : index(index), center(center), rotation(rot) 
        { /*    */ }

        CameraPose(int index, float x, float y, float z, float yaw, float pitch, float roll) : index(index)
        {
            center[0] = x;
            center[1] = y;
            center[2] = z;
            // FIXME generate rotation matrix
        }


    } CameraPose;

    typedef CameraPose* CameraPosePtr;
    typedef std::vector<CameraPose> CameraPoseList;
    




} // namesapce cameval

#endif // EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_


