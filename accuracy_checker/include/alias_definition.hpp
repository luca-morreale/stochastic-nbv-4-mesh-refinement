
#ifndef MESH_ACCURACY_ALIAS_DEFINITION_H
#define MESH_ACCURACY_ALIAS_DEFINITION_H

#include <glm/glm.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv/cv.hpp>

#include <manifoldReconstructor/types_reconstructor.hpp>

#include <set>
#include <vector>

typedef std::vector<int> IntList;
typedef std::vector<double> DoubleList;
typedef std::vector<IntList> IntArrayList;

typedef std::vector<cv::Vec2f> CVList2DVec;

typedef glm::vec3 GLMVec3;
typedef glm::vec2 GLMVec2;
typedef std::vector<GLMVec2> GLMList2DVec;
typedef std::vector<std::vector<GLMVec2>> GLMListArray2DVec;
typedef std::vector<GLMVec3> GLMList3DVec;

typedef std::vector<std::map<int, glm::vec2>> ListMappingGLM2DVec;


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



typedef std::vector<CameraType> CameraList;
typedef glm::mat4 CameraMatrix;
typedef std::vector<CameraMatrix> CameraMatrixList;

struct VisibilityInfo {
    GLMList3DVec points;                                // list of 3D points
    CameraMatrixList cameras;                           // list of cameras

    GLMListArray2DVec camObservations;                  // list of 2D points for each cam
    ListMappingGLM2DVec point3DTo2DThroughCam;     // list of points 3D to 2D through cam

};

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
