#ifndef CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
#define CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H

#include <CGAL/Vector_3.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <glm/glm.hpp>

#include <ilcplex/ilocplexi.h>

#include <map>
#include <set>
#include <vector>

#include <realtimeMR/types_reconstructor.hpp>

namespace opview {

    /* Shortcuts for GLM types */
    typedef glm::vec3 GLMVec3;
    typedef std::vector<GLMVec3> GLMVec3List;

    /* Shortcuts for Eigen types */
    typedef Eigen::VectorXd EigVector;
    typedef Eigen::Vector2d EigVector2;
    typedef Eigen::Vector3d EigVector3;
    typedef Eigen::Vector4d EigVector4;
    typedef Eigen::MatrixXd EigMatrix;
    typedef Eigen::Matrix3d EigMatrix3;
    typedef Eigen::Matrix4d EigMatrix4;
    typedef Eigen::Matrix<double, 3, 4> EigCameraMatrix;

    typedef std::vector<EigVector3> EigVector3List;
    typedef std::vector<EigVector> EigVectorList;
    typedef std::vector<EigMatrix> EigMatrixList;

    inline EigMatrix EigIdentity(int n) { return Eigen::MatrixXd::Identity(n, n); };

    inline EigMatrix EigZeros(int n) { return Eigen::MatrixXd::Zero(n, n); };
    inline EigMatrix EigZeros(int rows, int cols) { return Eigen::MatrixXd::Zero(rows, cols); };

    inline EigMatrix EigOnes(int n) { return Eigen::MatrixXd::Ones(n, n); };
    inline EigMatrix EigOnes(int rows, int cols) { return Eigen::MatrixXd::Ones(rows, cols); };

    /* Shortcuts for CGAL types */
    typedef std::vector<PointD3> PointD3List;
    typedef Delaunay3::Cell_handle CGALCell;
    typedef Delaunay3::Vertex_handle CGALVertex;
    typedef std::vector<Delaunay3::Cell_handle> CGALCellList;
    typedef std::set<Delaunay3::Cell_handle> CGALCellSet;
    typedef std::array<CGALVertex, 3> CGALFace;

    typedef std::array<PointD3, 3> FaceVertices;
    typedef std::vector<FaceVertices> FaceVerticesList;

    typedef K::Direction_3 CGALDirection;
    typedef K::Vector_3 CGALVec3;
    typedef std::vector<CGALVec3> CGALVec3List;

    typedef std::vector<IloNumVar> IloNumVarList;
    typedef std::vector<IloNum> IloNumList;
    typedef std::vector<IloExpr> IloExprList;
    typedef std::vector<IloRange> IloRangeList;
    typedef std::vector<IloConstraint> IloConstraintList;

}

#endif // CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
