#ifndef MESH_ACCURACY_ALIAS_DEFINITION_H
#define MESH_ACCURACY_ALIAS_DEFINITION_H

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Vector_3.h>

#include <CGAL/Simple_cartesian.h>
// #include <CGAL/Cartesian.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <glm/glm.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

#include <map>
#include <set>
#include <vector>

#include <realtimeMR/types_reconstructor.hpp>

namespace meshac {

    typedef std::pair<double, double> DoublePair;
    typedef std::pair<int, int> IntPair;
    typedef std::vector<double> DoubleList;
    typedef std::vector<int> IntList;
    typedef std::vector<IntList> IntArrayList;
    typedef std::vector<std::string> StringList;
    typedef std::map<int, DoubleList> IntDoubleListMap;

    /* Shortcuts for OpenCV types */
    typedef cv::line_descriptor::KeyLine CVSegment;
    typedef cv::Mat CVMat;
    typedef cv::Point CVPoint;
    typedef cv::Point2f CVPoint2;
    typedef cv::Vec3f CVLine;
    typedef cv::Vec3f CVVector3;
    typedef cv::Vec4f CVVector4;
    typedef std::vector<cv::Vec4f> CVVec4List;
    typedef std::vector<CVLine> CVLineList;
    typedef std::vector<CVMat> CVMatList;
    typedef std::vector<CVSegment> CVSegmentList;

    /* Shortcuts for GLM types */
    typedef glm::mat3 RotationMatrix;
    typedef glm::mat4 CameraMatrix;
    typedef glm::vec2 GLMVec2;
    typedef glm::vec3 GLMVec3;
    typedef glm::vec4 GLMVec4;
    typedef std::vector<GLMVec2> GLMVec2List;
    typedef std::vector<GLMVec3> GLMVec3List;
    typedef std::vector<std::vector<GLMVec2>> GLMVec2ArrayList;

    typedef std::map<int, GLMVec2> CamToPointMap;
    typedef std::pair<int, GLMVec2> CamPointPair;
    typedef std::vector<CamToPointMap> ListMappingGLMVec2;

    /* Shortcuts for Eigen types */
    typedef Eigen::Matrix3d EigMatrix3;
    typedef Eigen::Matrix4d EigMatrix4;
    typedef Eigen::Matrix<double, 3, 2> EigJacobianMatrix;
    typedef Eigen::Matrix<double, 3, 4> EigCameraMatrix;
    typedef Eigen::MatrixXd EigMatrix;
    typedef Eigen::Vector2d EigVector2;
    typedef Eigen::Vector3d EigVector3;
    typedef Eigen::Vector4d EigVector4;
    typedef Eigen::VectorXd EigVector;

    typedef Eigen::JacobiSVD<EigMatrix> EigSVD;
    typedef std::vector<EigMatrix> EigMatrixList;
    typedef std::vector<EigVector3> EigVector3List;
    typedef std::vector<EigVector> EigVectorList;
    typedef std::map<int, EigMatrixList> IntEigMatrixListMap;

    inline EigMatrix EigIdentity(int n) { return Eigen::MatrixXd::Identity(n, n); };

    inline EigMatrix EigZeros(int n) { return Eigen::MatrixXd::Zero(n, n); };
    inline EigMatrix EigZeros(int rows, int cols) { return Eigen::MatrixXd::Zero(rows, cols); };

    inline EigMatrix EigOnes(int n) { return Eigen::MatrixXd::Ones(n, n); };
    inline EigMatrix EigOnes(int rows, int cols) { return Eigen::MatrixXd::Ones(rows, cols); };


    /* Shortcut for manifoldReconstructor types */
    typedef std::vector<CameraType> CameraList;
    typedef glm::mat4 CameraMatrix;
    typedef std::vector<CameraMatrix> CameraMatrixList;

    typedef CGAL::Simple_cartesian<double> TreeKernel;
    // typedef CGAL::Cartesian<double> TreeKernel;
    typedef TreeKernel::Point_3 Point;
    typedef TreeKernel::Vector_3 Vector;
    typedef TreeKernel::Segment_3 Segment;
    typedef TreeKernel::Ray_3 Ray;
    typedef CGAL::Polyhedron_3<TreeKernel> Polyhedron;
    
    typedef Polyhedron::Vertex_handle Vertex;
    typedef Polyhedron::Facet_iterator Facet_iterator;
    typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
    typedef TreeKernel::Triangle_3 Triangle;
    typedef std::vector<Triangle> TriangleList;
    typedef CGAL::AABB_triangle_primitive<TreeKernel, TriangleList::iterator> Primitive;
    typedef CGAL::AABB_traits<TreeKernel, Primitive> Traits;

    typedef CGAL::AABB_tree<Traits> Tree;
    typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;

    typedef Tree* TreePtr;

    typedef std::map<int, Point> IntPointMap;
    typedef std::map<Point, int> PointIntMap;
    typedef std::vector<Point> PointList;

} // namespace meshac

#endif // MESH_ACCURACY_TYPE_DEFINITION_H
