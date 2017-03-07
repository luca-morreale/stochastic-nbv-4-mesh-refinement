
#ifndef MESH_ACCURACY_CR_TUPLES_GENERATOR_H
#define MESH_ACCURACY_CR_TUPLES_GENERATOR_H

#include <manifoldReconstructor/SfMData.h>

#include <CrossRatioTuple.hpp>
#include <type_definition.hpp>
#include <utilities.hpp>

namespace meshac {

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator();
        CRTuplesGenerator(SfMData *data);
        ~CRTuplesGenerator();


        /*
         * Extracts quadruplets of collinear points for each image.
         */
        ListCrossRatioTupleSet determineTupleOfFourPoints(SfMData *data);
        ListCrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtaine by the given camera.
         */
        CrossRatioTupleSet determineTupleOfFourPointsForCam(SfMData *data, int camIndex);
        CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);


        /*
         * Setter and Getter for SfMData.
         */
        void setSfMData(SfMData *data);
        SfMData *getSfMData();


    private:
        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMList2DVec &points2D);
        CVList2DVec createLinesFromPoints(int imgHeight, int imgWidth, GLMList2DVec &points2D);
        IntArrayList generateCorrespondances(CVList2DVec &lines, GLMList2DVec &points2D);
        void createFeatureImage(cv::Mat logicalImg, GLMList2DVec &points2D);

        SfMData *data;

    };


} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
