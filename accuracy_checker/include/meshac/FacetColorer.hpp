#ifndef MESH_ACCURACY_FACET_COLORER_H
#define MESH_ACCURACY_FACET_COLORER_H

#include <fstream>
#include <sstream>

#include <rapidjson/document.h>

#include <meshac/alias_definition.hpp>
#include <meshac/Colorer.hpp>
#include <meshac/FaceAccuracyModel.hpp>
#include <meshac/InvalidJsonFileException.hpp>
#include <meshac/OffParser.hpp>
#include <meshac/ThresholdColor.hpp>
#include <meshac/UnexpectedPointException.hpp>
#include <meshac/UnexpectedTriangleException.hpp>

namespace meshac {

    class FacetColorer : public Colorer {
    public:
        FacetColorer(std::string &confiFileName, FaceAccuracyModelPtr uncertantyEstimator);
        ~FacetColorer();

        virtual void generateColoredMesh(std::string &output);
        virtual void generateReport(std::string &output);

    protected:
        virtual Color getColorForFacet(int facetIndex);

        virtual double computeAccuracyForFacet(Point p1, Point p2, Point p3);
        virtual double computeAccuracyForFacet(Triangle &facet);
        virtual double computeAccuracyForFacet(int i);

        virtual Point computeBarycenter(const Point &p1, const Point &p2, const Point &p3);
        virtual Vector computeNormal(const Point &a, const Point &b, const Point &c);

        virtual void initOffFile(std::iostream &out, size_t points, size_t facets);
        virtual void addPointToOff(std::iostream &out, const Point &p);
        virtual void addFaceToOff(std::iostream &out, int v1, int v2, int v3, Color &color);
        virtual void addFaceToOff(std::iostream &out, int v1, int v2, int v3, std::string &color);

        virtual void addEntryToReport(std::iostream &out, const Point &a, const Point &b, const Point &c, double accuracy);

        virtual bool isSteinerFacet(int facetIndex);

    private:
        std::string configFileName;
        int facetsComputed = 0;

        FaceAccuracyModelPtr uncertantyEstimator;
    };

    typedef FacetColorer* FacetColorerPtr;

}

#endif // MESH_ACCURACY_FACET_COLORER_H
