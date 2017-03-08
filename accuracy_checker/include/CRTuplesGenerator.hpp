
#ifndef MESH_ACCURACY_CR_TUPLES_GENERATOR_H
#define MESH_ACCURACY_CR_TUPLES_GENERATOR_H

#include <manifoldReconstructor/SfMData.h>

#include <CrossRatioTuple.hpp>
#include <alias_definition.hpp>
#include <meshac_type_definition.hpp>
#include <utilities.hpp>

namespace meshac {

    class CRTuplesGenerator {
    public:
        CRTuplesGenerator();
        CRTuplesGenerator(SfMData *data);
        ~CRTuplesGenerator();


        /*
         * Setter and Getter for SfMData.
         */
        void setSfMData(SfMData *data);
        SfMData *getSfMData();


        /**
         * Getter for already computed CrossRatioTupleSet.
         */
        CrossRatioTupleSet getComputedTuples();
        CrossRatioTupleSet getComputedTuplesForCam(int camIndex);


        /*
         * Extracts quadruplets of collinear points for each image.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPoints(SfMData *data);
        virtual CrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtained by the given camera.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(SfMData *data, int camIndex);
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);


        
    protected:
        void setTuples(CrossRatioTupleSet tupleSet);
        void setTuplesPerCam(ListCrossRatioTupleSet tupleSetPerCam);

    private:
        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMList2DVec &points2D);
        CVList2DVec createLinesFromPoints(int imgHeight, int imgWidth, GLMList2DVec &points2D);
        IntArrayList generateCorrespondances(CVList2DVec &lines, GLMList2DVec &points2D);
        void createFeatureImage(cv::Mat logicalImg, GLMList2DVec &points2D);

        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet tupleSetPerCam);


        SfMData *data;
        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

    };


} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
