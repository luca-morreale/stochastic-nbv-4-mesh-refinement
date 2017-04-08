#ifndef MESH_ACCURACY_MESH_COLORER_H
#define MESH_ACCURACY_MESH_COLORER_H

#include <fstream>
#include <sstream>

#include <meshac/alias_definition.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>
#include <meshac/ThresholdColor.hpp>

namespace meshac {

    class MeshColorer {
    public:
        MeshColorer(std::string &confiFileName, Point3DVarianceEstimatorPtr uncertantyEstimator);
        ~MeshColorer();

        virtual Color getColorForPoint(GLMVec3 &point);
        virtual std::string printVertexColor(GLMVec3 &point);
        virtual void printVertex(std::stringstream &stream, GLMVec3 &point);

        virtual Color getColorForPoint(int pointIndex);
        virtual std::string printVertexColor(int pointIndex);
        virtual void printVertex(std::stringstream &stream, int pointIndex);

        std::string getConfigFileName();
        void setConfigFilename(std::string &confiFileName);

    protected:
        virtual void readColors();

    private:
        std::string fileName;

        Point3DVarianceEstimatorPtr uncertantyEstimator;

        ThresholdColorPtr colors;
    };

    typedef MeshColorer * MeshColorerPtr;

}

#endif // MESH_ACCURACY_MESH_COLORER_H
