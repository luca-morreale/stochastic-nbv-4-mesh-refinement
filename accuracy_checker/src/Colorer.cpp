#include <meshac/Colorer.hpp>

namespace meshac {

    Colorer::Colorer(std::string &confiFileName)
    {
        this->configFileName = confiFileName;
        this->colors = new ThresholdColor();
        this->readColors();
    }

    Colorer::~Colorer()
    {
        delete this->colors;
    }

    Color Colorer::getColorForAccuracy(double accuracy)
    {
        return colors->getColorFor(accuracy);
    }

    std::string Colorer::stringfyColor(Color &color)
    {
        double r = (double)color.r / 255.0;
        double g = (double)color.g / 255.0;
        double b = (double)color.b / 255.0;

        return std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b) + " " + std::to_string(color.a); 
    }

    void Colorer::readColors()
    {
        rapidjson::Document document = this->getJsonDocument();

        this->colors->clearColors();
        
        if (!document.IsObject()) throw InvalidJsonFileException("Invalid format for color file. \nRoot element is not an object.");
        if (!document.HasMember("colors")) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' does not exists.");
        
        const rapidjson::Value& colorsArray = document["colors"];

        if (!colorsArray.IsArray()) throw InvalidJsonFileException("Invalid format for color file. \nElement 'colors' is not an array.");

        
        this->extractColors(colorsArray);
    }

    void Colorer::extractColors(const rapidjson::Value& colors)
    {
        for (rapidjson::SizeType i = 0; i < colors.Size(); i++) {  // Uses SizeType instead of size_t
            if (this->hasCorrectMembers(colors[i])) {
                Color c = this->buildColor(colors[i]);
                this->colors->addColor(colors[i]["threshold"].GetFloat(), c);
            }
        }
    }

    rapidjson::Document Colorer::getJsonDocument()
    {
        std::ifstream jsonStream(this->getConfigFileName());
        std::string str((std::istreambuf_iterator<char>(jsonStream)), std::istreambuf_iterator<char>());

        rapidjson::Document document;
        document.Parse(str.c_str());
        return document;
    }

    bool Colorer::hasCorrectMembers(const rapidjson::Value& color)
    {
        return color.HasMember("threshold") && color.HasMember("r") && 
                color.HasMember("g") && color.HasMember("b") && color.HasMember("a");
    }

    Color Colorer::buildColor(const rapidjson::Value& color)
    {
        return {(byte)color["r"].GetInt(), (byte)color["g"].GetInt(), (byte)color["b"].GetInt(), color["a"].GetFloat()};
    }

    std::string Colorer::getConfigFileName()
    {
        return this->configFileName;
    }

    void Colorer::setConfigFilename(std::string &confiFileName)
    {
        this->configFileName = confiFileName;
        this->readColors();
    }

} // namespace meshac
