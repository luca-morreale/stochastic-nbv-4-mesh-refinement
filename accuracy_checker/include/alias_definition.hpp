
#ifndef MESH_ACCURACY_ALIAS_DEFINITION_H
#define MESH_ACCURACY_ALIAS_DEFINITION_H

#include <glm/glm.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv/cv.hpp>

#include <set>
#include <vector>

typedef std::vector<int> IntList;
typedef std::vector<IntList> IntArrayList;

typedef std::vector<cv::Vec2f> CVList2DVec;
typedef std::vector<glm::vec2> GLMList2DVec;

typedef Eigen::VectorXf EigVector;
typedef Eigen::MatrixXf EigMatrix;


inline EigMatrix EigIdentity(int n) { return Eigen::MatrixXf::Identity(n, n); };
inline EigMatrix EigZeros(int n) { return Eigen::MatrixXf::Zero(n, n); };




#endif // MESH_ACCURACY_TYPE_DEFINITION_H
