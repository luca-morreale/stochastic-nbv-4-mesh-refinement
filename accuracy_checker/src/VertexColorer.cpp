#include <meshac/VertexColorer.hpp>

namespace meshac {

    VertexColorer::VertexColorer(std::string &confiFileName, Point3DVarianceEstimatorPtr uncertantyEstimator) : Colorer(confiFileName)
    {
        this->uncertantyEstimator = uncertantyEstimator;
    }

    VertexColorer::~VertexColorer()
    {
        delete this->uncertantyEstimator;
    }

    Color VertexColorer::getColorForPoint(GLMVec3 &point)
    {
        double accuracy = this->uncertantyEstimator->computeSingleVarianceForPoint(point);
        return getColorForAccuracy(accuracy);
    }

    Color VertexColorer::getColorForPoint(int pointIndex)
    {
        double accuracy = this->uncertantyEstimator->computeSingleVarianceForPoint(pointIndex);
        return getColorForAccuracy(accuracy);
    }

    std::string VertexColorer::printVertexColor(GLMVec3 &point)
    {
        Color color = this->getColorForPoint(point);
        return stringfyColor(color);
    }

    std::string VertexColorer::printVertexColor(int pointIndex)
    {
        Color color = this->getColorForPoint(pointIndex);
        return stringfyColor(color);
    }

    void VertexColorer::printVertex(std::stringstream &stream, GLMVec3 &point)
    {
        stream << this->printVertexColor(point);
    }

    void VertexColorer::printVertex(std::stringstream &stream, int pointIndex)
    {
        stream << this->printVertexColor(pointIndex);
    }

} // namespace meshac
