#ifndef MESH_ACCURACY_MESH_COLORER_H
#define MESH_ACCURACY_MESH_COLORER_H

#include <fstream>
#include <sstream>

#include <rapidjson/document.h>

#include <meshac/alias_definition.hpp>
#include <meshac/InvalidJsonFileException.hpp>
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

        virtual bool hasCorrectMembers(const rapidjson::Value& color);
        virtual Color buildColor(const rapidjson::Value& color);

        virtual void extractColors(const rapidjson::Value& colors);

    

    private:
        std::string fileName;
        Point3DVarianceEstimatorPtr uncertantyEstimator;
        ThresholdColorPtr colors;

        rapidjson::Document getJsonDocument();
    };

    typedef MeshColorer * MeshColorerPtr;

}

#endif // MESH_ACCURACY_MESH_COLORER_H
