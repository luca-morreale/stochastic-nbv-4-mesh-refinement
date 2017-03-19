#ifndef MESH_ACCURACY_ALIAS_DEFINITION_H
#define MESH_ACCURACY_ALIAS_DEFINITION_H

#include <Eigen/Dense>
#include <Eigen/Core>

#include <glm/glm.hpp>

#include <opencv/cv.hpp>

#include <map>
#include <set>
#include <vector>

#include <manifoldReconstructor/types_reconstructor.hpp>


typedef std::vector<int> IntList;
typedef std::vector<double> DoubleList;
typedef std::vector<IntList> IntArrayList;

/* Shortcuts for OpenCV types */
typedef std::vector<cv::Vec2f> CVListVec2;
typedef cv::Mat CVMat;

/* Shortcuts for GLM types */
typedef glm::vec3 GLMVec3;
typedef glm::vec2 GLMVec2;
typedef std::vector<GLMVec2> GLMListVec2;
typedef std::vector<std::vector<GLMVec2>> GLMListArrayVec2;
typedef std::vector<GLMVec3> GLMListVec3;

typedef std::vector<std::map<int, glm::vec2>> ListMappingGLMVec2;

/* Shortcuts for Eigen types */
typedef Eigen::VectorXf EigVector;
typedef Eigen::Vector3f EigVector3;
typedef Eigen::Vector4f EigVector4;
typedef Eigen::MatrixXf EigMatrix;
typedef Eigen::Matrix3f EigMatrix3;
typedef Eigen::Matrix4f EigMatrix4;
typedef Eigen::Matrix<float, 3, 4> EigCameraMatrix;
typedef Eigen::Matrix<float, 3, 2> EigJacobianMatrix;

typedef std::vector<EigMatrix> EigMatrixList;

inline EigMatrix EigIdentity(int n) { return Eigen::MatrixXf::Identity(n, n); };

inline EigMatrix EigZeros(int n) { return Eigen::MatrixXf::Zero(n, n); };
inline EigMatrix EigZeros(int rows, int cols) { return Eigen::MatrixXf::Zero(rows, cols); };

inline EigMatrix EigOnes(int n) { return Eigen::MatrixXf::Ones(n, n); };
inline EigMatrix EigOnes(int rows, int cols) { return Eigen::MatrixXf::Ones(rows, cols); };


/* Shortcut for manifoldReconstructor types */
typedef std::vector<CameraType> CameraList;
typedef glm::mat4 CameraMatrix;
typedef std::vector<CameraMatrix> CameraMatrixList;


#endif // MESH_ACCURACY_TYPE_DEFINITION_H
