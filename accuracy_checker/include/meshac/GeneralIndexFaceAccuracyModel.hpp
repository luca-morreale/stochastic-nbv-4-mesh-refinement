#ifndef MESH_ACCURACY_GENERAL_INDEX_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_GENERAL_INDEX_FACE_ACCURACY_MODEL_H

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <iostream>

#include <meshac/camera_utilities.hpp>
#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/SfMData.h>
#include <meshac/type_definition.hpp>
#include <meshac/UndefinedFaceIndexException.hpp>
#include <meshac/UnexpectedPointException.hpp>
#include <meshac/UnexpectedTriangleException.hpp>
#include <meshac/utilities.hpp>

namespace meshac {

    // #define TRIANGLE_SIZE 24.5  // area of 100
    #define TRIANGLE_SIDE 500  // area of 4330.13
    
    class GeneralIndexFaceAccuracyModel : public FaceAccuracyModel {
    public:
        GeneralIndexFaceAccuracyModel(std::string &meshFile, SfMData &data, std::string &pathPrefix);
        ~GeneralIndexFaceAccuracyModel();

        /*
         * Computes all the matrixs that represents the accuracy of the point.
         */
        virtual double getAccuracyForFace(int faceIndex);
        virtual double getAccuracyForFace(Point &a, Point &b, Point &c);

    protected:
        void fixImagesPath(std::string &pathPrefix);
        void initMap3DTo2D();
        void initAffineTriangle();
        void setTriangularMask();

        virtual ListMappingGLMVec2 getMappings(int faceIndex);
        virtual IntList selectCommonCameras(ListMappingGLMVec2 &mappings);
        virtual ListMappingGLMVec2 removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams);
        virtual CVMat generateAffineTransform(GLMVec2 &a, GLMVec2 &b, GLMVec2 &c);
        virtual CVMat applyAffine(CVMat &affine, int camIndex);
        virtual CVMatList projectTriangles(ListMappingGLMVec2 &mappings, IntList &commonCams);

        virtual double computeIndex(CVMatList triangles) = 0;
        virtual double worstIndex() = 0;

        int retreiveIndex(Point &vertex);
        int retreiveIndex(GLMVec3 &point);

    private:
        StringList imgFilepath;

        PointIntMap pointToIndex;
        CameraList cams;
        GLMVec2ArrayList camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;

        TreePtr tree;
        FaceIndexList faces;
        CVPoint2 destTriangle[3];
        CVMat triangularMask;


        const GLMVec4 zdir = GLMVec4(0.0, 0.0, 1.0, 0.0);

        IntList unionCamIndex(ListMappingGLMVec2 &mappings);
        CVMat cropTriangle(CVMat &projectedImage);

    };

    typedef GeneralIndexFaceAccuracyModel* GeneralIndexFaceAccuracyModelPtr;

}

#endif // MESH_ACCURACY_GENERAL_INDEX_FACE_ACCURACY_MODEL_H
