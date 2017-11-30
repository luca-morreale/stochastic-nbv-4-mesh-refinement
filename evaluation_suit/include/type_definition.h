#ifndef EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_
#define EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_

#include <aliases.h>

namespace cameval {

    typedef struct Camera {
        GLMMat4 P;
        GLMMat4 R;
        GLMMat4 E;
        GLMMat4 K;
        GLMVec4 t;

        Camera(GLMMat4 R, GLMMat4 K, GLMVec4 t) : R(R), K(K)
        {
            E = R;
            E[0][3] = t.x;
            E[1][3] = t.y;
            E[2][3] = t.z;
            E[3][3] = 1.0f;
            P = E * K;
            this->t = - R * t;
        }

    } Camera;

    typedef std::vector<Camera> CameraList;

    struct CameraType {
        long unsigned int idCam;
        long int idReconstruction = -1;

        GLMMat3 intrinsics;
        GLMMat3 rotation;
        GLMVec3 translation;
        GLMMat4 cameraMatrix;
        GLMVec3 center;
        GLMMat4 mvp;

        std::string pathImage;

        int imageWidth;
        int imageHeight;
    };

    typedef std::vector<CameraType> CameraTypeList;
    

} // namesapce cameval

#endif // EVALUATION_CAMERA_POSITION_TYPE_DEFINITION_H_
