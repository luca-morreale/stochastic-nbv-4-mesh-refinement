#ifndef MESH_ACCURACY_MESH_COLORER_H
#define MESH_ACCURACY_MESH_COLORER_H

#include <fstream>
#include <sstream>

#include <meshac/alias_definition.hpp>
#include <meshac/ThresholdColor.hpp>

namespace meshac {

    class MeshColorer {
    public:
        MeshColorer(std::string confiFileName);
        ~MeshColorer();

        std::string printVertexColor(GLMVec3 &point, double &accuracy);
        std::string printVertex(GLMVec3 &point, double &accuracy);
        void printVertex(std::stringstream &stream, GLMVec3 &point, double &accuracy);
        void encodeVertexToStream(std::stringstream &stream, GLMListVec3 &points, DoubleList &accuracies);

        std::string getConfigFileName();
        void setConfigFilename(std::string confiFileName);

    protected:
        virtual void readColors();

    private:
        std::string fileName;

        ThresholdColorPtr colors;
    };

    typedef MeshColorer * MeshColorerPtr;

}

#endif // MESH_ACCURACY_MESH_COLORER_H
