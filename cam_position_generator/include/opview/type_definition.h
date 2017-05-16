#ifndef CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
#define CAM_POSITION_GENERATOR_TYPE_DEFINITION_H

#include <map>

#include <opview/alias_definition.h>

#include <opengm/graphicalmodel/graphicalmodel.hxx>
#include <opengm/graphicalmodel/space/simplediscretespace.hxx>
#include <opengm/functions/sparsemarray.hxx>
#include <opengm/operations/adder.hxx>
#include <opengm/operations/multiplier.hxx>
#include <opengm/inference/graphcut.hxx>
#include <opengm/inference/auxiliary/minstcutkolmogorov.hxx>
#include <opengm/inference/alphaexpansion.hxx>
#include <opengm/inference/alphabetaswap.hxx>
#include <opengm/inference/icm.hxx>
#include <opengm/inference/lazyflipper.hxx>
#include <opengm/inference/loc.hxx>


namespace opview {

    struct Face {
        FaceVertices face;
        PointD3 oppositeVertex;
    };

    typedef std::vector<Face> FaceList;

    typedef double LabelType;
    typedef size_t VariableIndexType;

    typedef std::vector<LabelType> LabelList;


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

    typedef opengm::external::MinSTCutKolmogorov<size_t, double> MinStCutType;
    typedef opengm::GraphCut<GraphicalModelAdder, opengm::Maximizer, MinStCutType> MinGraphCut;
    typedef opengm::AlphaExpansion<GraphicalModelAdder, MinGraphCut> MinAlphaExpansion;
    typedef opengm::AlphaBetaSwap<GraphicalModelAdder, MinGraphCut> MinAlphaBetaSwap;
    typedef opengm::ICM<GraphicalModelAdder, opengm::Maximizer> ICM;
    typedef opengm::LazyFlipper<GraphicalModelAdder, opengm::Maximizer> LazyFlipper;
    typedef opengm::LOC<GraphicalModelAdder, opengm::Maximizer> LOC;


    typedef struct VonMisesConfiguration {
        double goalAngle;
        double dispersion;

    public:
        VonMisesConfiguration(double goalAngle, double dispersion) : goalAngle(goalAngle), dispersion(dispersion) { }
        VonMisesConfiguration() : goalAngle(45), dispersion(0) { }
    } VonMisesConfiguration;

    typedef VonMisesConfiguration * VonMisesConfigurationPtr;

}

#endif // CAM_POSITION_GENERATOR_TYPE_DEFINITION_H
