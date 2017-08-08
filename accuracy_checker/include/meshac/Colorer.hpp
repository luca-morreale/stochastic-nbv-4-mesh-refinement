#ifndef MESH_ACCURACY_COLORER_H
#define MESH_ACCURACY_COLORER_H

#include <fstream>
#include <sstream>

#include <rapidjson/document.h>

#include <meshac/alias_definition.hpp>
#include <meshac/InvalidJsonFileException.hpp>
#include <meshac/ThresholdColor.hpp>

namespace meshac {

    class Colorer {
    public:
        Colorer(std::string &confiFileName);
        ~Colorer();

        std::string getConfigFileName();
        virtual void setConfigFilename(std::string &confiFileName);

    protected:
        virtual void readColors();

        virtual std::string stringfyColor(Color &color);
        virtual Color getColorForAccuracy(double accuracy);

    private:
        std::string configFileName;
        ThresholdColorPtr colors;

        rapidjson::Document getJsonDocument();
        bool hasCorrectMembers(const rapidjson::Value& color);
        Color buildColor(const rapidjson::Value& color);
        void extractColors(const rapidjson::Value& colors);
    };

    typedef Colorer* ColorerPtr;

}

#endif // MESH_ACCURACY_COLORER_H
