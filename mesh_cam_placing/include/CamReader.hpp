#ifndef CENTERS_READER_H
#define CENTERS_READER_H

#include <rapidjson/document.h>
#include <rapidjson/reader.h>

#include <iostream>
#include <fstream>
#include <cmath>

#include <aliases.h>


namespace camplacing {
    
    class CamReader {
    public:
        CamReader(std::string &camfile);

        virtual GLMVec3List parse();
        GLMVec3List getCameras();
        GLMMat3List getRots();

    private:
        std::string camfile;
        GLMVec3List cams;
        GLMMat3List rots;

        GLMMat3 getRotationMatrix(double pitch, double yaw);
        double radians(double deg);
    };

} // namespace camplacing

#endif // CENTERS_READER_H
