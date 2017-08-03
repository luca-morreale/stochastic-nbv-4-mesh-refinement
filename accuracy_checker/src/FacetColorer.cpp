#include <meshac/FacetColorer.hpp>

namespace meshac {

    FacetColorer::FacetColorer(std::string &confiFileName, FaceAccuracyModelPtr uncertantyEstimator)
    {
        this->fileName = confiFileName;
        this->uncertantyEstimator = uncertantyEstimator;
        this->colors = new ThresholdColor();
        this->readColors();
    }

    FacetColorer::~FacetColorer()
    {
        delete this->uncertantyEstimator;
        delete this->colors;
    }

    void FacetColorer::generateColoredMesh(std::string &output)
    {
        std::ofstream out(output);
        TriangleList facets = uncertantyEstimator->getFaces();

        std::map<Point, int> pointToIndex = extractPointToIndexMap(facets);

        initOffFile(out, pointToIndex.size(), facets.size());   // before this I should know the number of points and the number of vertex
        
        for (auto pair : pointToIndex) {
            addPointToOff(out, pair.first);
        }
        
        for (int f = 0; f < facets.size(); f++) {
            
            try {
                double accuracy = computeAccuracyForFacet(facets[f]);

                Color color = colors->getColorFor(accuracy);

                addFaceToOff(out, facets[f].vertex(0), facets[f].vertex(1), facets[f].vertex(2), pointToIndex, color);
            } catch (UnexpectedTriangleException &ex) { }
        }

        out.close();
    }

    double FacetColorer::computeAccuracyForFacet(Triangle &facet)
    {
        try {
            const Point v1 = facet.vertex(0);
            const Point v2 = facet.vertex(1);
            const Point v3 = facet.vertex(2);

            GLMVec3 a = GLMVec3(v1.x(), v1.y(), v1.z());
            GLMVec3 b = GLMVec3(v2.x(), v2.y(), v2.z());
            GLMVec3 c = GLMVec3(v3.x(), v3.y(), v3.z());

            return uncertantyEstimator->getAccuracyForFace(a, b, c);

        } catch (UnexpectedPointException &ex) { }
        return 1.0;
    }

    std::map<Point, int> FacetColorer::extractPointToIndexMap(TriangleList &facets)
    {
        std::map<Point, int> pointToIndex;
        int index = 1;

        for (int f = 0; f < facets.size(); f++) {
            for (int v = 0; v < 3; v++) {
                Point vertex = facets[f].vertex(v);
                if (pointToIndex[vertex] != 0) {
                    pointToIndex[vertex] = index++;
                }
            }
        }
        return pointToIndex;
    }

    void FacetColorer::initOffFile(std::ofstream &out, size_t points, size_t facets)
    {
        out << "OFF" << std::endl;
        out << points << " " << facets << std::endl;
    }

    void FacetColorer::addPointToOff(std::ofstream &out, const Point &p)
    {
        out << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }

    void FacetColorer::addFaceToOff(std::ofstream &out, const Point &p1, const Point &p2, const Point &p3, std::map<Point, int> &pointToIndex, Color &color)
    {
        out << pointToIndex[p1] << " " << pointToIndex[p2] << " " << pointToIndex[p3] << "  ";
        out << color.to_string() << std::endl;
    }

    void FacetColorer::generateReport(std::string &output)
    {
        std::ofstream out(output);
        TriangleList facets = uncertantyEstimator->getFaces();
        
        for (int f = 0; f < facets.size(); f++) {

            double accuracy = computeAccuracyForFacet(facets[f]);

            addEntryToReport(out, facets[f].vertex(0), facets[f].vertex(1), facets[f].vertex(2), accuracy);
        }
        out.close();
    }

    void FacetColorer::addEntryToReport(std::ofstream &out, const Point &p1, const Point &p2, const Point &p3, double accuracy)
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

    void FacetColorer::readColors()
    {
        rapidjson::Document document = this->getJsonDocument();

        this->colors->clearColors();
        
        if (!document.IsObject()) throw InvalidJsonFileException("Invalid format for color file. \nRoot element is not an object.");
        if (!document.HasMember("colors")) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' does not exists.");
        
        const rapidjson::Value& colorsArray = document["colors"];

        if (!colorsArray.IsArray()) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' is not an array.");

        
        this->extractColors(colorsArray);
    }

    void FacetColorer::extractColors(const rapidjson::Value& colors)
    {
        for (rapidjson::SizeType i = 0; i < colors.Size(); i++) {  // Uses SizeType instead of size_t
            if (this->hasCorrectMembers(colors[i])) {
                Color c = this->buildColor(colors[i]);
                this->colors->addColor(colors[i]["threshold"].GetFloat(), c);
            }
        }
    }

    rapidjson::Document FacetColorer::getJsonDocument()
    {
        std::ifstream jsonStream(this->fileName);
        std::string str((std::istreambuf_iterator<char>(jsonStream)), std::istreambuf_iterator<char>());

        rapidjson::Document document;
        document.Parse(str.c_str());
        return document;
    }

    bool FacetColorer::hasCorrectMembers(const rapidjson::Value& color)
    {
        return color.HasMember("threshold") && color.HasMember("r") && 
                color.HasMember("g") && color.HasMember("b") && color.HasMember("a");
    }

    Color FacetColorer::buildColor(const rapidjson::Value& color)
    {
        return {(byte)color["r"].GetInt(), (byte)color["g"].GetInt(), (byte)color["b"].GetInt(), color["a"].GetFloat()};
    }

    std::string FacetColorer::getConfigFileName()
    {
        return this->fileName;
    }

    void FacetColorer::setConfigFilename(std::string &confiFileName)
    {
        this->fileName = confiFileName;
        this->readColors();
    }

} // namespace meshac
