#ifndef UTILITIES_HPP_
#define UTILITIES_HPP_

#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <glm/glm.hpp>
#include <string>

#include <type_definition.h>

namespace utilities {


/*Stores the visibility rays in a ply file named initVisT.ply
 *
 *
 * param camCenters list of camera centers in world coordinate
 * param points list of points centers in camera coordinates
 * pointsVisibleFromCamN visibility information: vector of i int vectors; each i-th int vectors is the list
 *                       of points indices  visible from th ei-th cam
 */
void saveVisibilityPly(const std::vector<glm::vec3> &camCenters, const std::vector<glm::vec3> & points,
    const std::vector<std::vector<int> >& visibility,const std::string &name = "visibility", bool pointsVisibleFromCamN = true);



/*computes rotation around X axis*/
glm::mat3 rotX(float alpha);

/*computes rotation around Y axis*/
glm::mat3 rotY(float alpha);

/*computes rotation around Z axis*/
glm::mat3 rotZ(float alpha);

/**
 * Read one line in the configuration file and stores the parameter in the value variable
 */
void readLineAndStore(std::ifstream &configFile, bool &value);
void readLineAndStore(std::ifstream &configFile, int &value);
void readLineAndStore(std::ifstream &configFile, double &value);
void readLineAndStore(std::ifstream &configFile, float &value);
void readLineAndStore(std::ifstream &configFile, std::string &value);

typedef struct AccuracyScore {
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    std::vector<double> uncertainty;
} AccuracyScore;

AccuracyScore readScores(std::string accScore);

}  // namespace utils

#endif /* UTILITIES_HPP_ */
