#ifndef CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
#define CAM_POSITION_GENERATOR_TYPE_DEFINITION_H

#include <map>

#include <opview/alias_definition.h>

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


namespace opview {

    struct Face {
        FaceVertices face;
        PointD3 oppositeVertex;
    };

    typedef std::vector<Face> FaceList;

    typedef float LabelType;
    typedef size_t VariableIndexType;

    typedef std::vector<LabelType> LabelList;
    typedef std::vector<size_t> VarIndexList;


    typedef opengm::SimpleDiscreteSpace<VariableIndexType, LabelType> SimpleSpace;
    typedef opengm::ExplicitFunction<LabelType> GMExplicitFunction;
    typedef opengm::SparseFunction<LabelType, VariableIndexType, LabelType> GMSparseFunction;
    typedef OPENGM_TYPELIST_2(GMExplicitFunction, GMSparseFunction) FunctionTypeList;
    typedef opengm::GraphicalModel<LabelType, opengm::Adder, FunctionTypeList, SimpleSpace> GraphicalModelAdder;
    typedef opengm::GraphicalModel<LabelType, opengm::Multiplier, FunctionTypeList, SimpleSpace> GraphicalModelMultiplier;

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
    

    typedef struct VonMisesConfiguration {
        double goalAngle;
        double dispersion;

    public:
        VonMisesConfiguration(double goalAngle, double dispersion) : goalAngle(goalAngle), dispersion(dispersion) { }
        VonMisesConfiguration() : goalAngle(45), dispersion(0) { }
    } VonMisesConfiguration;

    typedef VonMisesConfiguration * VonMisesConfigurationPtr;

    typedef struct HierarchicalDiscretizationConfiguration {
        size_t depth;
        size_t labels;

    public:
        HierarchicalDiscretizationConfiguration(size_t depth, size_t labels) : depth(depth), labels(labels) { }
        HierarchicalDiscretizationConfiguration() : depth(5), labels(10) { }

    } HierarchicalDiscretizationConfiguration;

    typedef HierarchicalDiscretizationConfiguration* HierarchicalDiscretizationConfigPtr;

    typedef struct OrientationHierarchicalConfiguration {
        HierarchicalDiscretizationConfiguration config;
        float deltaAngle;

    public:
        OrientationHierarchicalConfiguration(size_t depth, size_t labels, float deltaAngle) : deltaAngle(deltaAngle)
        {
            config = {depth, labels};
        }
        OrientationHierarchicalConfiguration() : deltaAngle(10)
        {
            config = HierarchicalDiscretizationConfiguration();
        }

    } OrientationHierarchicalConfiguration;

    typedef OrientationHierarchicalConfiguration* OrientationHierarchicalConfigPtr;


}

#endif // CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
