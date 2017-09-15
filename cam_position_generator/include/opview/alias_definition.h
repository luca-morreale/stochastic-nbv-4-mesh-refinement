#ifndef CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
#define CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H

#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Vector_3.h>
#include <CGAL/Point_3.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <glm/glm.hpp>

#include <boost/function.hpp> 
#include <boost/bind.hpp>

#include <map>
#include <set>
#include <vector>
#include <queue>

#include <opengm/functions/sparsemarray.hxx>
#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/inference/alphabetaswap.hxx>
#include <opengm/inference/alphaexpansion.hxx>
#include <opengm/inference/auxiliary/minstcutkolmogorov.hxx>
#include <opengm/inference/graphcut.hxx>
#include <opengm/inference/icm.hxx>
#include <opengm/inference/lazyflipper.hxx>
#include <opengm/inference/loc.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/operations/multiplier.hxx>

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

    typedef std::pair<GLMVec3List, GLMVec3List> GLMVec3ListPair;

    /* Shortcuts for Eigen types */
    typedef Eigen::Matrix<float, 5, 1> EigVector5;
    typedef std::vector<EigVector5> EigVector5List;

    typedef std::pair<double, EigVector5> ValuePose;
    struct ComparePoses{
        bool operator()(ValuePose const& lhs, ValuePose const& rhs){
            return lhs.first > rhs.first;
        }
    };

    typedef std::priority_queue<ValuePose, std::vector<ValuePose>, ComparePoses> OrderedPose;


    typedef std::vector<gsl_vector *> GSLVectorList;
    typedef std::vector<gsl_matrix *> GSLMatrixList;


    /* Shortcuts for CGAL types */
    typedef CGAL::Simple_cartesian<double> TreeKernel;
    typedef TreeKernel::Point_3 Point;
    typedef TreeKernel::Vector_3 Vector;
    typedef std::vector<Vector> VectorList;
    typedef TreeKernel::Segment_3 Segment;
    typedef TreeKernel::Ray_3 Ray;
    typedef CGAL::Polyhedron_3<TreeKernel> Polyhedron;

    typedef std::vector<Point> PointList;

    typedef Polyhedron::Vertex_handle Vertex;
    typedef Polyhedron::Facet_iterator Facet_iterator;
    typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
    typedef TreeKernel::Triangle_3 Triangle;
    typedef std::vector<Triangle> TriangleList;
    typedef TriangleList::iterator Iterator;
    typedef CGAL::AABB_triangle_primitive<TreeKernel, Iterator> Primitive;
    typedef CGAL::AABB_traits<TreeKernel, Primitive> Traits;

    typedef CGAL::AABB_tree<Traits> Tree;
    typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
    typedef Tree::Primitive_id Primitive_id;

    typedef Tree* TreePtr;

    typedef std::function<float()> LambdaFloat;
    typedef std::function<GLMVec3()> LambdaGLMVec3;
    typedef std::function<GLMVec3List(OrderedPose&)> LambdaGLMPointsList;
    typedef std::function<EigVector5List(OrderedPose&)> LambdaEigPointsList;
    

    /* OpenGM aliases */
    typedef double LabelType;
    typedef size_t VariableIndexType;

    typedef std::vector<LabelType> LabelList;
    typedef std::vector<size_t> VarIndexList;


    typedef opengm::DiscreteSpace<> SimpleSpace;
    typedef opengm::ExplicitFunction<LabelType, size_t, VariableIndexType> GMExplicitFunction;
    typedef opengm::SparseFunction<LabelType, VariableIndexType, LabelType> GMSparseFunction;
    typedef OPENGM_TYPELIST_2(GMExplicitFunction, GMSparseFunction) FunctionTypeList;
    typedef opengm::GraphicalModel<LabelType, opengm::Adder, FunctionTypeList, SimpleSpace> GraphicalModelAdder;
    typedef opengm::GraphicalModel<LabelType, opengm::Multiplier, FunctionTypeList, SimpleSpace> GraphicalModelMultiplier;

    typedef std::vector<GMSparseFunction> GMSparseFunctionList;
    typedef std::vector<GMExplicitFunction> GMExplicitFunctionList;
    
    typedef GraphicalModelAdder::FunctionIdentifier GMAdderFID;
    typedef GraphicalModelMultiplier::FunctionIdentifier GMMultFID;

    typedef SimpleSpace * SimpleSpacePtr;
    typedef GraphicalModelAdder * GraphicalModelAdderPtr;
    typedef GraphicalModelMultiplier * GraphicalModelMultiplierPtr;

    typedef opengm::Inference<GraphicalModelAdder, opengm::Maximizer> AdderInference;
    typedef opengm::Inference<GraphicalModelMultiplier, opengm::Maximizer> MultiplierInference;
    typedef opengm::external::MinSTCutKolmogorov<size_t, double> MinStCutType;
    typedef opengm::GraphCut<GraphicalModelAdder, opengm::Maximizer, MinStCutType> MinGraphCut;
    typedef opengm::AlphaExpansion<GraphicalModelAdder, MinGraphCut> MinAlphaExpansion;
    typedef opengm::AlphaBetaSwap<GraphicalModelAdder, MinGraphCut> MinAlphaBetaSwap;
    typedef opengm::ICM<GraphicalModelAdder, opengm::Maximizer> ICM;
    typedef opengm::LazyFlipper<GraphicalModelAdder, opengm::Maximizer> LazyFlipper;
    typedef LazyFlipper::Parameter LazyFlipperParameter;
    typedef opengm::LOC<GraphicalModelAdder, opengm::Maximizer> LOC;
    typedef opengm::Bruteforce<GraphicalModelAdder, opengm::Maximizer> Bruteforce;

    typedef AdderInference* AdderInferencePtr;
    typedef MultiplierInference* MultiplierInferencePtr;
    typedef MinAlphaExpansion* MinAlphaExpansionPtr;
    typedef MinAlphaBetaSwap* MinAlphaBetaSwapPtr;
    typedef ICM* ICMPtr;
    typedef LazyFlipper* LazyFlipperPtr;
    typedef LOC* LOCPtr;
    typedef Bruteforce* BruteforcePtr;
}

#endif // CAM_POSITION_GENERATOR_ALIAS_DEFINITION_H
