#ifndef MESH_ACCURACY_VERTEX_COLORER_H
#define MESH_ACCURACY_VERTEX_COLORER_H

#include <fstream>
#include <sstream>

#include <rapidjson/document.h>

#include <meshac/alias_definition.hpp>
#include <meshac/Colorer.hpp>
#include <meshac/InvalidJsonFileException.hpp>
#include <meshac/Point3DVarianceEstimator.hpp>
#include <meshac/ThresholdColor.hpp>

namespace meshac {

    class VertexColorer : public Colorer {
    public:
        VertexColorer(std::string &confiFileName, Point3DVarianceEstimatorPtr uncertantyEstimator);
        virtual ~VertexColorer();

        virtual Color getColorForPoint(GLMVec3 &point);
        virtual std::string printVertexColor(GLMVec3 &point);
        virtual void printVertex(std::stringstream &stream, GLMVec3 &point);

        virtual Color getColorForPoint(int pointIndex);
        virtual std::string printVertexColor(int pointIndex);
        virtual void printVertex(std::stringstream &stream, int pointIndex);
    

    private:
        Point3DVarianceEstimatorPtr uncertantyEstimator;
    };

    typedef VertexColorer* VertexColorerPtr;

}

#endif // MESH_ACCURACY_VERTEX_COLORER_H
