#ifndef MESH_ACCURACY_FACE_ACCURACY_MODEL_H
#define MESH_ACCURACY_FACE_ACCURACY_MODEL_H

#include <meshac/alias_definition.hpp>
#include <meshac/type_definition.hpp>

namespace meshac {
    
    class FaceAccuracyModel {
    public:
        FaceAccuracyModel(std::string &meshFile);
        virtual ~FaceAccuracyModel();

        /*
         * Computes all the matrixs that represents the accuracy of the point.
         */
        virtual double getAccuracyForFace(int indexFace) = 0;
        virtual double getAccuracyForFace(GLMVec3 &a, GLMVec3 &b, GLMVec3 &c) = 0;

        void changeMesh(std::string meshFile);
        TriangleList getFaces();

    protected:
        virtual TriangleList generateTriangleList();
    

    

    private:
        std::string meshFilename;
        TriangleList faces;

        Polyhedron extractPolyhedron();

    };

}

#endif // MESH_ACCURACY_FACE_ACCURACY_MODEL_H
