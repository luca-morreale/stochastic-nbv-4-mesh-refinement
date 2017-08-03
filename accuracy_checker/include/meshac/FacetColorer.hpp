#ifndef MESH_ACCURACY_FACET_COLORER_H
#define MESH_ACCURACY_FACET_COLORER_H

#include <fstream>
#include <sstream>

#include <rapidjson/document.h>

#include <meshac/alias_definition.hpp>
#include <meshac/InvalidJsonFileException.hpp>
#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/ThresholdColor.hpp>
#include <meshac/UnexpectedPointException.hpp>
#include <meshac/UnexpectedTriangleException.hpp>

namespace meshac {

    class FacetColorer {
    public:
        FacetColorer(std::string &confiFileName, FaceAccuracyModelPtr uncertantyEstimator);
        virtual ~FacetColorer();

        virtual void generateColoredMesh(std::string &output);
        virtual void generateReport(std::string &output);

        virtual std::string getConfigFileName();
        virtual void setConfigFilename(std::string &confiFileName);

    protected:
        virtual void readColors();

        virtual bool hasCorrectMembers(const rapidjson::Value& color);
        virtual Color buildColor(const rapidjson::Value& color);

        virtual void extractColors(const rapidjson::Value& colors);

        virtual double computeAccuracyForFacet(Triangle &facet);

        virtual Point computeBarycenter(const Point &p1, const Point &p2, const Point &p3);
        virtual Vector computeNormal(const Point &a, const Point &b, const Point &c);

        virtual std::map<Point, int> extractPointToIndexMap(TriangleList &facets);
        virtual void initOffFile(std::ofstream &out, size_t points, size_t facets);
        virtual void addPointToOff(std::ofstream &out, const Point &p);
        virtual void addFaceToOff(std::ofstream &out, const Point &p1, const Point &p2, const Point &p3, std::map<Point, int> &pointToIndex, Color &color);

        virtual void addEntryToReport(std::ofstream &out, const Point &a, const Point &b, const Point &c, double accuracy);

    private:
        std::string fileName;
        FaceAccuracyModelPtr uncertantyEstimator;
        ThresholdColorPtr colors;

        rapidjson::Document getJsonDocument();
    };

    typedef FacetColorer * FacetColorerPtr;

}

#endif // MESH_ACCURACY_FACET_COLORER_H
