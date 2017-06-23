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
        GLMVec3 getBestCamera();
        GLMVec3 getFinalCamera();

        GLMMat3List getRotations();
        GLMMat3 getRotationOfBest();
        GLMMat3 getRotationOfFinal();

    private:
        std::string camfile;
        DoubleList scores;
        GLMVec3List cams;
        GLMMat3List rots;

        int bestScoreIndex();
        GLMMat3 createRotationMatrix(double pitch, double yaw);
        double radians(double deg);
    };

} // namespace camplacing

#endif // CENTERS_READER_H
