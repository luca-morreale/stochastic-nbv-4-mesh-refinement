#ifndef CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
#define CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Vector_3.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <glm/glm.hpp>

#include <boost/function.hpp> 
#include <boost/bind.hpp>

#include <ilcplex/ilocplexi.h>

#include <map>
#include <set>
#include <vector>
#include <queue>

#include <realtimeMR/types_reconstructor.hpp>

namespace opview {

    typedef std::map<double, int> DoubleIntMap;
    typedef std::map<int, double> IntDoubleMap;
    typedef std::vector<double> DoubleList;
    typedef std::vector<DoubleIntMap> DoubleIntMapList;
    typedef std::vector<float> FloatList;
    typedef std::vector<int> IntList;
    typedef std::vector<IntDoubleMap> IntDoubleMapList;
    typedef std::vector<size_t> SizeTList;

    typedef std::pair<double, double> DoublePair;
    typedef std::pair<double, int> DoubleIntPair;
    typedef std::set<double> DoubleSet;
    typedef std::vector<DoubleIntPair> DoubleIntList;
    typedef std::vector<DoublePair> DoublePairList;
    typedef std::vector<DoubleSet> DoubleSetList;
    typedef DoubleSet::iterator DoubleSetIterator;

    /* Shortcuts for GLM types */
    typedef glm::mat3 RotationMatrix;
    typedef glm::mat4 CameraMatrix;
    typedef glm::vec2 GLMVec2;
    typedef glm::vec3 GLMVec3;
    typedef glm::vec4 GLMVec4;
    typedef std::vector<GLMVec3> GLMVec3List;

    /* Shortcuts for Eigen types */
    typedef Eigen::Matrix<float, 5, 1> EigVector5;
    typedef std::vector<EigVector5> EigVector5List;

    typedef std::pair<double, EigVector5> ValuePose;
    struct ComparePoses{
        bool operator()(ValuePose const& lhs, ValuePose const& rhs){
            return lhs.first < rhs.first;
        }
    };

    typedef std::priority_queue<ValuePose, std::vector<ValuePose>, ComparePoses> OrderedPose;


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
    typedef CGAL::Polyhedron_3<K> Polyhedron;
    typedef Polyhedron::Vertex_handle Vertex_handle;
    typedef Polyhedron::Facet_iterator Facet_iterator;
    typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
    typedef K::Triangle_3 Triangle;
    typedef std::vector<Triangle> TriangleList;
    typedef TriangleList::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
    typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
    typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
    typedef Tree* TreePtr;
    
    typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;

    typedef std::vector<IloNumVar> IloNumVarList;
    typedef std::vector<IloNum> IloNumList;
    typedef std::vector<IloExpr> IloExprList;
    typedef std::vector<IloRange> IloRangeList;
    typedef std::vector<IloConstraint> IloConstraintList;

    typedef std::vector<gsl_vector *> GSLVectorList;
    typedef std::vector<gsl_matrix *> GSLMatrixList;

    typedef boost::function<double(EigVector5 &, GLMVec3 &, GLMVec3 &)> BoostObjFunction;
    typedef std::vector<BoostObjFunction> BoostObjFunctionList;

    typedef std::function<float()> LambdaFloat;
    typedef std::function<GLMVec3List(OrderedPose&)> LambdaGLMPointsList;
    typedef std::function<EigVector5List(OrderedPose&)> LambdaEigPointsList;

}

#endif // CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
