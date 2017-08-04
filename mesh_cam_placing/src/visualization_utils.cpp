#include <visualization_utils.hpp>

namespace camplacing {

    // x    white y    blue z    yellow
    const StringList axesColor = {("1.0 1.0 1.0 0.75"), ("0.0 0.0 1.0 0.75"), ("1.0 1.0 0.0 0.75")};

    void generateJsonFromData(GLMVec3List &cams, GLMVec3List &defaultCams, GLMMat3List &rots, GLMMat3List &defaultRots, std::string &outname)
    {
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

        
        std::ofstream off(outname + ".off");
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
    }

    void generateJsonFromData(GLMVec3List &cams, GLMVec3List &defaultCams, GLMMat3List &rots, GLMMat3List &defaultRots, std::string &outname, GLMVec3 &poi)
    {
        CubeList cubes, defaultCubes;
        AxesList axes, defaultAxes;
        GLMVec3List points, defaultPoints;

        auto sphere = readSphere();

        appendCube(cams, points, cubes);
        appendOrientation(cams, points, axes, rots);
        appendCube(defaultCams, defaultPoints, defaultCubes);
        appendOrientation(defaultCams, defaultPoints, defaultAxes, defaultRots);

        int offset = points.size();
        size_t pointQt = points.size() + defaultPoints.size() + sphere.first.size();
        size_t cubeQt = cubes.size() + defaultCubes.size();

        size_t faces = (cubeQt * 6) + axes.size() + defaultAxes.size() + sphere.second.size();
        size_t edges = (cubeQt * 12) + axes.size() * 3 + defaultAxes.size() * 3 + 3 * sphere.second.size();

        
        std::ofstream off(outname + ".off");
        off << "OFF" << std::endl;
        off << pointQt << " " << faces << " " << 0 << std::endl;
        
        printPoints(off, points);
        printPoints(off, defaultPoints);
        printSpherePoints(off, sphere.first, poi);

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

        printSphereFacets(off, sphere.second, offset + defaultPoints.size());
        
        off.close();
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
        off << "4  " << cube[0]+offset << "  " << cube[4]+offset << "  " << cube[5]+offset << "  " << cube[1]+offset << "  " << color << std::endl;
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
        return cam + R * v;
    }

} // namespace camplacing
