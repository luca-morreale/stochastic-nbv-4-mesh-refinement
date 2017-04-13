
#include <meshac/MeshColorer.hpp>

namespace meshac {

    MeshColorer::MeshColorer(std::string &confiFileName, Point3DVarianceEstimatorPtr uncertantyEstimator)
    {
        this->fileName = confiFileName;
        this->uncertantyEstimator = uncertantyEstimator;
        this->colors = new ThresholdColor();
        this->readColors();
    }

    MeshColorer::~MeshColorer()
    {
        delete this->uncertantyEstimator;
        delete this->colors;
    }

    Color MeshColorer::getColorForPoint(GLMVec3 &point)
    {
        double accuracy = this->uncertantyEstimator->computeVarianceForPoint(point);
        return this->colors->getColorFor(accuracy);
    }

    Color MeshColorer::getColorForPoint(int pointIndex)
    {
        double accuracy = this->uncertantyEstimator->computeVarianceForPoint(pointIndex);
        return this->colors->getColorFor(accuracy);
    }

    std::string MeshColorer::printVertexColor(GLMVec3 &point)
    {
        Color color = this->getColorForPoint(point);
        return color.to_string();
    }

    std::string MeshColorer::printVertexColor(int pointIndex)
    {
        Color color = this->getColorForPoint(pointIndex);
        return color.to_string();
    }

    void MeshColorer::printVertex(std::stringstream &stream, GLMVec3 &point)
    {
        stream << this->printVertexColor(point);
    }

    void MeshColorer::printVertex(std::stringstream &stream, int pointIndex)
    {
        stream << this->printVertexColor(pointIndex);
    }

    void MeshColorer::readColors()
    {
        rapidjson::Document document = this->getJsonDocument();

        this->colors->clearColors();
        
        if (!document.IsObject()) throw InvalidJsonFileException("Invalid format for color file. \nRoot element is not an object.");
        if (!document.HasMember("colors")) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' does not exists.");
        
        const rapidjson::Value& colorsArray = document["colors"];

        if (!colorsArray.IsArray()) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' is not an array.");

        
        this->extractColors(colorsArray);
    }

    void MeshColorer::extractColors(const rapidjson::Value& colors)
    {
        for (rapidjson::SizeType i = 0; i < colors.Size(); i++) {  // Uses SizeType instead of size_t
            if (this->hasCorrectMembers(colors[i])) {
                Color c = this->buildColor(colors[i]);
                this->colors->addColor(colors[i]["threshold"].GetFloat(), c);
            }
        }
    }

    rapidjson::Document MeshColorer::getJsonDocument()
    {
        std::ifstream jsonStream(this->fileName);
        std::string str((std::istreambuf_iterator<char>(jsonStream)), std::istreambuf_iterator<char>());

        rapidjson::Document document;
        document.Parse(str.c_str());
        return document;
    }

    bool MeshColorer::hasCorrectMembers(const rapidjson::Value& color)
    {
        return color.HasMember("threshold") && color.HasMember("r") && 
                color.HasMember("g") && color.HasMember("b") && color.HasMember("a");
    }

    Color MeshColorer::buildColor(const rapidjson::Value& color)
    {
        return {(byte)color["r"].GetInt(), (byte)color["g"].GetInt(), (byte)color["b"].GetInt(), color["a"].GetFloat()};
    }

    std::string MeshColorer::getConfigFileName()
    {
        return this->fileName;
    }

    void MeshColorer::setConfigFilename(std::string &confiFileName)
    {
        this->fileName = confiFileName;
        this->readColors();
    }

}
