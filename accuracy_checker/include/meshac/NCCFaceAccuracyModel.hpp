#ifndef MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H

#include <algorithm>

#include <realtimeMR/SfMData.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/type_definition.hpp>
#include <meshac/UnexpectedPointException.hpp>
#include <meshac/UnexpectedTriangleException.hpp>
#include <meshac/utilities.hpp>

namespace meshac {

    #define TRIANGLE_SIZE 10
    
    class NCCFaceAccuracyModel : public FaceAccuracyModel {
    public:
        NCCFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix);
        ~NCCFaceAccuracyModel();

        /*
         * Computes all the matrixs that represents the accuracy of the point.
         */
        virtual double getAccuracyForFace(int faceIndex);
        virtual double getAccuracyForFace(GLMVec3 &a, GLMVec3 &b, GLMVec3 &c);

    protected:
        virtual IntList selectCommonCameras(ListMappingGLMVec2 &mappings);

        virtual CVMat generateAffineTransform(GLMVec2 &a, GLMVec2 &b, GLMVec2 &c);
        virtual CVMat applyAffine(CVMat &affine, int camIndex);
        virtual CVMatList projectTriangles(ListMappingGLMVec2 &mappings, IntList &commonCams);
        virtual double computeNCC(CVMatList triangles);

    private:
        GLMVec3List points;
        
        StringList imgFilepath;

        CameraMatrixList cameras;
        GLMVec2ArrayList camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;

        FaceIndexList faces;
        CVPoint2 destTriangle[3];

        void initAffineTriangle();

        void fixImagesPath(std::string &pathPrefix);
        CameraMatrixList extractCameraMatrix(CameraList &cameras);
        void convertTriangleToIndex();

        size_t retreiveIndex(PointD3 &vertex);
        size_t retreiveIndex(GLMVec3 &point);
        ListMappingGLMVec2 getMappings(int faceIndex);
        void removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams);
        IntList unionCamIndex(ListMappingGLMVec2 &mappings);


    };

}

#endif // MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
