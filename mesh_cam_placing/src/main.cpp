
#include <realtimeMR/CameraPointsCollection.h>
#include <realtimeMR/ConfigParser.h>
#include <realtimeMR/ReconstructFromSfMData.h>
#include <realtimeMR/ReconstructFromSLAMData.h>
#include <realtimeMR/types_config.hpp>
#include <realtimeMR/types_reconstructor.hpp>
#include <realtimeMR/utilities/Chronometer.h>
#include <realtimeMR/utilities/Logger.h>

#include <boost/circular_buffer.hpp>

#include <iostream>
#include <cmath>

#include <OpenMvgParser.h>
#include <CamReader.hpp>
#include <aliases.h>

#define CUBE_SIZE 0.05

using namespace camplacing;

SfMData sfm_data_;

GLMVec3 rotatePoint(GLMVec3 point, GLMVec3 &cam, GLMMat3 &R);

void appendCube(GLMVec3List &cams, GLMVec3List &points, CubeList &cubes);
void appendOrientation(GLMVec3List &cams, GLMVec3List &points, AxesList &axes, GLMMat3List &rots);
void printCube(std::ofstream &off, Cube &cube, int offset, std::string color);
void printPoints(std::ofstream &off, GLMVec3List &points);
void printAxes(std::ofstream &off, Axes &ax, int offset, std::string color);


int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "missing arguments" << std::endl;
        return 1;
    }

    omp_set_num_threads(8);

    std::string input_file, cams_file;

    int maxIterations_ = 0;

    input_file = argv[2];
    cams_file = argv[1];
    std::cout << "Using default configuration res/config/default.json" << std::endl;
    std::cout << "max_iterations not set" << std::endl << std::endl;

    std::cout << "parsing" << std::endl;
    OpenMvgParser op_openmvg(input_file);
    op_openmvg.parse();
    std::cout << "sfm: " << op_openmvg.getSfmData().numCameras_ << " cams; " << op_openmvg.getSfmData().numPoints_ << " points" << std::endl << std::endl;

    sfm_data_ = op_openmvg.getSfmData();

    auto tmpCams = sfm_data_.camerasList_;
    GLMVec3List defaultCams;
    GLMMat3List defaultRots;

    for (auto cam : tmpCams) {
        defaultCams.push_back(cam.center);
        defaultRots.push_back(cam.rotation);
    }

    std::vector<std::string> axesColor;
    axesColor.push_back("1.0 1.0 1.0 0.75");    // x    white
    axesColor.push_back("0.0 0.0 1.0 0.75");    // y    blue
    axesColor.push_back("1.0 1.0 0.0 0.75");    // z    yellow

    std::cout<<"parsed \n";
    CamReader reader(cams_file);
    GLMVec3List cams = reader.parse();
    GLMMat3List rots = reader.getRots();

    CubeList cubes, defaultCubes;
    AxesList axes, defaultAxes;
    GLMVec3List points, defaultPoints;

    appendCube(cams, points, cubes);
    appendOrientation(cams, points, axes, rots);
    appendCube(defaultCams, defaultPoints, defaultCubes);
    appendOrientation(defaultCams, defaultPoints, defaultAxes, defaultRots);

    int offset = points.size();
    size_t pointQt = points.size() + defaultPoints.size();
    size_t cubeQt = cubes.size()+defaultCubes.size();

    size_t faces = (cubeQt * 6) + axes.size() + defaultAxes.size();
    size_t edges = (cubeQt * 12) + axes.size() * 3 + defaultAxes.size() * 3;

    std::ofstream off("../cams_file.off");
    off << "OFF" << std::endl;
    off << pointQt << " " << faces << " " << edges << std::endl;
    
    printPoints(off, points);
    printPoints(off, defaultPoints);

    for (Cube cube : cubes) {
        printCube(off, cube, 0, "1.0 0.0 0.0 0.75");
    }
    for (Cube cube : defaultCubes) {
        printCube(off, cube, offset, "0.0 1.0 0.0 0.75");
    }

    int i = 0;
    for (Axes ax : axes) {
        printAxes(off, ax, 0, axesColor[i%3]);
        i++;
    }
    i=0;
    for (Axes ax : defaultAxes) {
        printAxes(off, ax, offset, axesColor[i%3]);
        i++;
    }
    
    off.close();

    return 0;
}

void appendCube(GLMVec3List &cams, GLMVec3List &points, CubeList &cubes)
{
    double shift = CUBE_SIZE / 2.0;

    for (GLMVec3 cam : cams) {
        Cube c;
        int i = 0;
        for (int x = -1; x <= 1; x+=2) {
            for (int y = -1; y <= 1; y+=2) {
                for (int z = -1; z <= 1; z+=2) {
                    points.push_back(GLMVec3(cam.x + shift * x, cam.y + shift * y, cam.z + shift * z));
                    c[i++] = points.size()-1;
                }
            }
        }
        cubes.push_back(c);
    }
}

void appendOrientation(GLMVec3List &cams, GLMVec3List &points, AxesList &axes, GLMMat3List &rots)
{
    // do rotation!!!
    double shift = CUBE_SIZE;

    for (int i = 0; i < cams.size(); i++) {
        GLMVec3 cam = cams[i];
        GLMMat3 R = rots[i];

        
        points.push_back(cam);
        int camIndex = points.size() - 1;

        Axes ax;
        points.push_back(rotatePoint(GLMVec3(cam.x + shift, cam.y, cam.z-shift/3), cam, R));
        points.push_back(rotatePoint(GLMVec3(cam.x + shift, cam.y, cam.z+shift/3), cam, R));
        ax[0] = camIndex;
        ax[1] = points.size()-2;
        ax[2] = points.size()-1;
        axes.push_back(ax);

        ax = Axes();
        points.push_back(rotatePoint(GLMVec3(cam.x, cam.y+shift, cam.z-shift/3), cam, R));
        points.push_back(rotatePoint(GLMVec3(cam.x, cam.y+shift, cam.z+shift/3), cam, R));
        ax[0] = camIndex;
        ax[1] = points.size()-2;
        ax[2] = points.size()-1;
        axes.push_back(ax);

        ax = Axes();
        points.push_back(rotatePoint(GLMVec3(cam.x-shift/3, cam.y, cam.z + shift), cam, R));
        points.push_back(rotatePoint(GLMVec3(cam.x+shift/3, cam.y, cam.z + shift), cam, R));
        ax[0] = camIndex;
        ax[1] = points.size()-2;
        ax[2] = points.size()-1;
        axes.push_back(ax);

    }
}

void printPoints(std::ofstream &off, GLMVec3List &points)
{
    for (GLMVec3 point : points) {
        off << point.x << "  " << point.y << "  " << point.z << std::endl;
    }
}

void printCube(std::ofstream &off, Cube &cube, int offset, std::string color)
{
    off << "4  " << cube[0]+offset << "  " << cube[1]+offset << "  " << cube[3]+offset << "  " << cube[2]+offset << "  " << color << std::endl;
    off << "4  " << cube[0]+offset << "  " << cube[4]+offset << "  " << cube[5]+offset << "  " << cube[3]+offset << "  " << color << std::endl;
    off << "4  " << cube[4]+offset << "  " << cube[6]+offset << "  " << cube[7]+offset << "  " << cube[5]+offset << "  " << color << std::endl;
    off << "4  " << cube[0]+offset << "  " << cube[4]+offset << "  " << cube[6]+offset << "  " << cube[2]+offset << "  " << color << std::endl;
    off << "4  " << cube[2]+offset << "  " << cube[6]+offset << "  " << cube[7]+offset << "  " << cube[3]+offset << "  " << color << std::endl;
    off << "4  " << cube[1]+offset << "  " << cube[5]+offset << "  " << cube[7]+offset << "  " << cube[3]+offset << "  " << color << std::endl;
}

void printAxes(std::ofstream &off, Axes &ax, int offset, std::string color)
{
    off << "3  " << ax[0]+offset << "  " << ax[1]+offset << "  " << ax[2]+offset << "  " << color << std::endl;
}

GLMVec3 rotatePoint(GLMVec3 point, GLMVec3 &cam, GLMMat3 &R)
{
    auto v = point - cam;
    return cam - R * v;
}
