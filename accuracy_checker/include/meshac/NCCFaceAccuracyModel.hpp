#ifndef MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H

#include <algorithm>

#include <CGAL/exceptions.h>

#include <realtimeMR/SfMData.h>

#include <glm/gtc/epsilon.hpp>
#include <glm/gtx/norm.hpp>

#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/type_definition.hpp>
#include <meshac/UnexpectedPointException.hpp>
#include <meshac/UnexpectedTriangleException.hpp>
#include <meshac/UndefinedFaceIndexException.hpp>
#include <meshac/UnprojectablePointThroughCamException.hpp>
#include <meshac/utilities.hpp>

namespace meshac {

    // #define TRIANGLE_SIZE 24.5  // area of 100
    #define TRIANGLE_SIZE 100  // area of 4330.13
    
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
        virtual void projectMeshPoints();
        virtual IntList selectCommonCameras(ListMappingGLMVec2 &mappings);

        virtual CVMat generateAffineTransform(GLMVec2 &a, GLMVec2 &b, GLMVec2 &c);
        virtual CVMat applyAffine(CVMat &affine, int camIndex);
        virtual CVMatList projectTriangles(ListMappingGLMVec2 &mappings, IntList &commonCams);
        virtual double computeNCC(CVMatList triangles);

        virtual GLMVec2 projectThrough(GLMVec3 &meshPoint, int camIndex);
        virtual bool isMeaningfulPose(GLMVec3 &meshPoint, int camIndex);
        virtual bool isOppositeView(GLMVec3 &centroid, int camIndex);
        virtual bool isIntersecting(GLMVec3 &meshPoint, int camIndex);
        virtual bool isMathemathicalError(Segment_intersection &intersection, PointD3 &point);
        virtual bool isPointInsideImage(GLMVec2 &point2D, int camIndex);
        virtual GLMVec2 getProjectedPoint(GLMVec3 &meshPoint, int camIndex);
        GLMVec3 getCameraCenter(int indexCam);
        RotationMatrix getRotationMatrix(int indexCam);
        CameraMatrix getCameraMatrix(int indexCam);

        int getImageWidth(int camIndex);
        int getImageHeight(int camIndex);

    private:
        StringList imgFilepath;

        GLMVec3List points;
        CameraList cams;
        GLMVec2ArrayList camObservations;
        ListMappingGLMVec2 point3DTo2DThroughCam;

        TreePtr tree;
        FaceIndexList faces;
        CVPoint2 destTriangle[3];

        const GLMVec3 zdir = GLMVec3(0.0, 0.0, 1.0);

        void initAffineTriangle();
        void initTree();

        void fixImagesPath(std::string &pathPrefix);
        void convertTriangleToIndex();

        int retreiveIndex(PointD3 &vertex);
        int retreiveIndex(GLMVec3 &point);
        ListMappingGLMVec2 getMappings(int faceIndex);
        ListMappingGLMVec2 removeUnusedMapping(ListMappingGLMVec2 &mappings, IntList &commonCams);
        IntList unionCamIndex(ListMappingGLMVec2 &mappings);

    };

    typedef NCCFaceAccuracyModel* NCCFaceAccuracyModelPtr;

}

#endif // MESH_ACCURACY_NCC_FACE_ACCURACY_MODEL_H
