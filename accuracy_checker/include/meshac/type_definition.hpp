#ifndef MESH_ACCURACY_TYPE_DEFINITION_H
#define MESH_ACCURACY_TYPE_DEFINITION_H

#include <vector>

namespace meshac {

    const float SENSIBILITY = 0.001f;

    typedef std::array<size_t, 3> SizeT3Array;

    typedef struct FaceIndex {
        SizeT3Array vs;

        FaceIndex() { /*    */ }
        FaceIndex(size_t x, size_t y, size_t z)
        {
            vs[0] = x;
            vs[1] = y;
            vs[2] = z;
        }

        void set(int pos, size_t index)
        {
            vs[pos] = index;
        }

        bool is(int a, int b, int c)
        {
            return (a == (int)vs[0] && b == (int)vs[1] && c == (int)vs[2]) ||
                (b == (int)vs[0] && c == (int)vs[1] && a == (int)vs[2]) ||
                (c == (int)vs[0] && a == (int)vs[1] && b == (int)vs[2]);
        }

    } FaceIndex;

    typedef std::vector<FaceIndex> FaceIndexList;

    /*
     * struct from manifoldReconstructor, repo of Andrea Romanoni
     */
    struct CameraType {
        long unsigned int idCam;
        long int idReconstruction = -1;

        glm::mat3 intrinsics;
        glm::mat3 rotation;
        glm::vec3 translation;
        glm::mat4 cameraMatrix;
        glm::vec3 center;
        glm::mat4 mvp;

        std::string pathImage;

        int imageWidth;
        int imageHeight;
    };

    typedef std::vector<CameraType> CameraList;
    typedef glm::mat4 CameraMatrix;
    typedef std::vector<CameraMatrix> CameraMatrixList;
    
} // namesace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
