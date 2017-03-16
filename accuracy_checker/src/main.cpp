
#include <manifoldReconstructor/OpenMvgParser.h>
#include <manifoldReconstructor/SfMData.h>
#include <manifoldReconstructor/types_reconstructor.hpp>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>


#include <CrossRatioTuple.hpp>
#include <CRTuplesGenerator.hpp>
#include <alias_definition.hpp>
#include <meshac_type_definition.hpp>



int main(int argc, char **argv) {
  
    OpenMvgParser op(argv[1]);
    op.parse();
    
    SfMData points = op.getSfmData();

    meshac::CRTuplesGenerator crGenerator = meshac::CRTuplesGenerator(points.point2DoncamViewingPoint_, points.imageWidth_, points.imageHeight_);

    meshac::CrossRatioTupleSet crossratioTupleSet = crGenerator.determineTupleOfFourPoints();


    return 0;
}



