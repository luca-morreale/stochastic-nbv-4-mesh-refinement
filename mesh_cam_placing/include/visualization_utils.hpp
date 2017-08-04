#ifndef CAM_PLACING_VISUALIZATION_H
#define CAM_PLACING_VISUALIZATION_H

#include <cmath>
#include <fstream>

#include <aliases.h>
#include <sphere_handler.hpp>

namespace camplacing {

    #define CUBE_SIZE 0.05

    void generateJsonFromData(GLMVec3List &cams, GLMVec3List &defaultCams, GLMMat3List &rots, GLMMat3List &defaultRots, std::string &outname);
    void generateJsonFromData(GLMVec3List &cams, GLMVec3List &defaultCams, GLMMat3List &rots, GLMMat3List &defaultRots, std::string &outname, GLMVec3 &poi);
        
    GLMVec3 rotatePoint(GLMVec3 point, GLMVec3 &cam, GLMMat3 &R);
    void appendCube(GLMVec3List &cams, GLMVec3List &points, CubeList &cubes);
    void appendOrientation(GLMVec3List &cams, GLMVec3List &points, AxesList &axes, GLMMat3List &rots);
    void printCube(std::ofstream &off, Cube &cube, int offset, std::string color);
    void printPoints(std::ofstream &off, GLMVec3List &points);
    void printAxes(std::ofstream &off, Axes &ax, int offset, std::string color);
    
} // namespace camplacing

#endif // CAM_PLACING_VISUALIZATION_H
