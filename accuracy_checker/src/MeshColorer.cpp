
#include <meshac/MeshColorer.hpp>

namespace meshac {

    MeshColorer::MeshColorer(std::string &confiFileName)
    {
        this->fileName = confiFileName;
        this->colors = new ThresholdColor();
        this->readColors();
    }

    MeshColorer::~MeshColorer()
    {
        delete this->colors;
    }

    std::string MeshColorer::printVertexColor(GLMVec3 &point, double &accuracy)
    {
        Color color = this->colors->getColorFor(accuracy);
        return " " + color.string();
    }

    std::string MeshColorer::printVertex(GLMVec3 &point, double &accuracy)
    {
        std::string out = std::to_string(point[0]) + " " + std::to_string(point[1]) + " " + std::to_string(point[2]);
        return out + " " + printVertexColor(point, accuracy);
    }

    void MeshColorer::printVertex(std::stringstream &stream, GLMVec3 &point, double &accuracy)
    {
        stream << this->printVertex(point, accuracy);
    }

    void MeshColorer::encodeVertexToStream(std::stringstream &stream, GLMListVec3 &points, DoubleList &accuracies)
    {
        for (int i = 0; i < points.size(); i++) {
            stream << this->printVertex(points[i], accuracies[i]) << std::endl;
        }
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
