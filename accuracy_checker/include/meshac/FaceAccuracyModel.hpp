#ifndef MESH_ACCURACY_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_FACE_ACCURACY_MODEL_H

#include <meshac/alias_definition.hpp>
#include <meshac/OffParser.hpp>
#include <meshac/type_definition.hpp>
#include <meshac/utilities.hpp>

namespace meshac {
    
    class FaceAccuracyModel {
    public:
        FaceAccuracyModel(std::string &meshFile);
        virtual ~FaceAccuracyModel();

        /*
         * Computes all the matrixs that represents the accuracy of the point.
         */
        virtual double getAccuracyForFace(int indexFace) = 0;
        virtual double getAccuracyForFace(Point &a, Point &b, Point &c) = 0;

        void setMeshFile(std::string meshFile);
        TriangleList getFaces();
        FaceIndexList getFacetsIndex();
        PointList getPoints();

    protected:
        virtual TriangleList generateTriangleList();
        virtual void initTree();

        TreePtr getTree();

    private:
        std::string meshFilename;

        TriangleList faces;
        FaceIndexList trianglesIndex;
        PointIntMap pointToIndex;

        TreePtr tree;
        
        Polyhedron extractPolyhedron();

    };

    typedef FaceAccuracyModel* FaceAccuracyModelPtr;

}

#endif // MESH_ACCURACY_FACE_ACCURACY_MODEL_H
