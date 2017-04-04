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
typedef std::vector<std::string> StringList;

/* Shortcuts for OpenCV types */
typedef cv::Vec2f CVVector2;
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
typedef Eigen::VectorXd EigVector;
typedef Eigen::Vector2d EigVector2;
typedef Eigen::Vector3d EigVector3;
typedef Eigen::Vector4d EigVector4;
typedef Eigen::MatrixXd EigMatrix;
typedef Eigen::Matrix3d EigMatrix3;
typedef Eigen::Matrix4d EigMatrix4;
typedef Eigen::Matrix<double, 3, 4> EigCameraMatrix;
typedef Eigen::Matrix<double, 3, 2> EigJacobianMatrix;

typedef std::vector<EigVector3> EigVector3List;
typedef std::vector<EigVector> EigVectorList;
typedef std::vector<EigMatrix> EigMatrixList;

inline EigMatrix EigIdentity(int n) { return Eigen::MatrixXd::Identity(n, n); };

inline EigMatrix EigZeros(int n) { return Eigen::MatrixXd::Zero(n, n); };
inline EigMatrix EigZeros(int rows, int cols) { return Eigen::MatrixXd::Zero(rows, cols); };

inline EigMatrix EigOnes(int n) { return Eigen::MatrixXd::Ones(n, n); };
inline EigMatrix EigOnes(int rows, int cols) { return Eigen::MatrixXd::Ones(rows, cols); };


/* Shortcut for manifoldReconstructor types */
typedef std::vector<CameraType> CameraList;
typedef glm::mat4 CameraMatrix;
typedef std::vector<CameraMatrix> CameraMatrixList;


#endif // MESH_ACCURACY_TYPE_DEFINITION_H
