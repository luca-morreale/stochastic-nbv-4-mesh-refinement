#ifndef CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
#define CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H

#include <CGAL/Vector_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_triangle_primitive.h>

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
    typedef glm::vec2 GLMVec2;
    typedef glm::vec3 GLMVec3;
    typedef glm::vec4 GLMVec4;
    typedef std::vector<GLMVec3> GLMVec3List;
    typedef glm::mat4 CameraMatrix;

    /* Shortcuts for Eigen types */
    typedef Eigen::Matrix<float, 5, 1> EigVector5;


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

    typedef K::Ray_3 Ray;
    typedef std::vector<Triangle>::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;

    typedef std::vector<IloNumVar> IloNumVarList;
    typedef std::vector<IloNum> IloNumList;
    typedef std::vector<IloExpr> IloExprList;
    typedef std::vector<IloRange> IloRangeList;
    typedef std::vector<IloConstraint> IloConstraintList;

}

#endif // CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
