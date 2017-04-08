
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
        return color.string();
    }

    std::string MeshColorer::printVertexColor(int pointIndex)
    {
        Color color = this->getColorForPoint(pointIndex);
        return color.string();
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
        this->colors->clearColors();

        std::ifstream cin(this->fileName);

        float threshold = 0;
        float r = 0, g = 0, b = 0;
        float a = 0;

        while (!cin.eof()) {
            cin >> threshold >> r >> g >> b >> a;
            Color c = {r, g, b, a};
            
            this->colors->addColor(threshold, c);
        }
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
