#include <meshac/FacetColorer.hpp>

namespace meshac {

    FacetColorer::FacetColorer(std::string &confiFileName, FaceAccuracyModelPtr uncertantyEstimator) : Colorer(confiFileName)
    {
        this->uncertantyEstimator = uncertantyEstimator;
    }

    FacetColorer::~FacetColorer()
    {
        delete this->uncertantyEstimator;
    }

    void FacetColorer::generateColoredMesh(std::string &output)
    {
        std::stringstream stream;
        
        PointList points = uncertantyEstimator->getPoints();
        FaceIndexList facets = uncertantyEstimator->getFacetsIndex();

        initOffFile(stream, points.size(), facets.size());

        for (int i = 0; i < points.size(); i++) {
            addPointToOff(stream, points[i]);
        }

        std::vector<Color> colors(facets.size());
        #pragma omp parallel for
        for (int i = 0; i < facets.size(); i++) {
            Color color = getColorForFacet(i);
            colors[i] = color;            
        }
        
        for (int i = 0; i < facets.size(); i++) {
            addFaceToOff(stream, facets[i].vs[0], facets[i].vs[1], facets[i].vs[2], colors[i]);
        }

        std::ofstream out(output);
        out << stream.str();
        out.close();
    }

    void FacetColorer::generateReport(std::string &output)
    {
        std::stringstream stream;
        std::ofstream out(output);

        PointList points = uncertantyEstimator->getPoints();
        FaceIndexList facets = uncertantyEstimator->getFacetsIndex();
        
        DoubleList accs(facets.size());
        #pragma omp parallel for
        for (int f = 0; f < facets.size(); f++) {
            double accuracy = computeAccuracyForFacet(f);
            accs[f] = accuracy;
        }

        for (int f = 0; f < facets.size(); f++) {
            addEntryToReport(stream, points[facets[f].vs[0]], points[facets[f].vs[1]], points[facets[f].vs[2]], accs[f]);
        }

        out << stream.str();
        out.close();
    }

    double FacetColorer::computeAccuracyForFacet(Point p1, Point p2, Point p3)
    {
        try {
            return uncertantyEstimator->getAccuracyForFace(p1, p2, p3);
        } catch (UnexpectedPointException &ex) { }
        return 1.0;
    }

    double FacetColorer::computeAccuracyForFacet(int i)
    {
        if (!isSteinerFacet(i)) {
            try {
                return uncertantyEstimator->getAccuracyForFace(i);
            } catch (UnexpectedPointException &ex) { }
        } else {
            return 1.0;     // low = accurate point, high = not accurate point.
        }
    }

    double FacetColorer::computeAccuracyForFacet(Triangle &facet)
    {
        try {
            Point v1 = facet.vertex(0);
            Point v2 = facet.vertex(1);
            Point v3 = facet.vertex(2);

            return uncertantyEstimator->getAccuracyForFace(v1, v2, v3);

        } catch (UnexpectedPointException &ex) { }
        return 1.0;
    }

    void FacetColorer::initOffFile(std::iostream &out, size_t points, size_t facets)
    {
        out << "OFF" << std::endl;
        out << points << " " << facets << " 0" << std::endl;
    }

    void FacetColorer::addPointToOff(std::iostream &out, const Point &p)
    {
        out << p.x() << " " << p.y() << " " << p.z() << std::endl;
    }

    void FacetColorer::addFaceToOff(std::iostream &out, int v1, int v2, int v3, Color &color)
    {
        out << "3 "<< v1 << " " << v2 << " " << v3 << "  " << stringfyColor(color) << std::endl;
    }

    void FacetColorer::addFaceToOff(std::iostream &out, int v1, int v2, int v3, std::string &color)
    {
        out << "3 "<< v1 << " " << v2 << " " << v3 << "  " << color << std::endl;
    }

    void FacetColorer::addEntryToReport(std::iostream &out, const Point &p1, const Point &p2, const Point &p3, double accuracy)
    {
        Point barycenter = computeBarycenter(p1, p2, p3);
        Vector normal = computeNormal(p1, p2, p3);

        out << barycenter.x() << " " << barycenter.y() << " " << barycenter.z() << " ";
        out << normal.x() << " " << normal.y() << " " << normal.z() << " " << accuracy << std::endl;
    }

    Point FacetColorer::computeBarycenter(const Point &p1, const Point &p2, const Point &p3)
    {
        return CGAL::barycenter(p1, 1, p2, 1, p3, 1);
    }

    Vector FacetColorer::computeNormal(const Point &p1, const Point &p2, const Point &p3)
    {
        return CGAL::normal(p1, p2, p3);
    }

    Color FacetColorer::getColorForFacet(int facetIndex)
    {
        try {
            double accuracy = computeAccuracyForFacet(facetIndex);
            return getColorForAccuracy(accuracy);
        } catch (const UnexpectedTriangleException &ex) {
            std::cerr << ex.what() << std::endl;
            return Color(1.0, 1.0, 1.0, 0.3);
        }
    }

    bool FacetColorer::isSteinerFacet(int facetIndex)
    {
        auto facet = uncertantyEstimator->getFaces()[facetIndex];
        // case of steiner facet
        return facet.squared_area() > 400.0;
    }

} // namespace meshac
