
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
        CRTuplesGenerator(GLMListArray2DVec camObservations, int obsWidth, int obsHeight);
        ~CRTuplesGenerator();


        /*
         * Getter and setter for Cameras' Observations.
         */
        GLMListArray2DVec getCamObservations();
        void setCamObservations(GLMListArray2DVec camObservations);
        void setCamObservations(GLMList2DVec list, int camIndex);
        void updateCamObservations(GLMList2DVec list, int camIndex);


        /*
         * Getter and setter for Size of Camera's Observations.
         */
        void setObsSize(int obsWidth, int obsHeight);
        std::pair<int,int> getObsSize();



        /*
         * Getter for already computed CrossRatioTupleSet.
         */
        CrossRatioTupleSet getComputedTuples();
        CrossRatioTupleSet getComputedTuplesForCam(int camIndex);
        ListCrossRatioTupleSet getCrossRatioTupleSetList();


        /*
         * Extracts quadruplets of collinear points for each image.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPoints(GLMListArray2DVec camObservations, int obsWidth, int obsHeight);
        virtual CrossRatioTupleSet determineTupleOfFourPoints();

        /*
         * Extracts quadruplets of collinear points for the image obtained by the given camera.
         */
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(GLMListArray2DVec camObservations, int camIndex, int obsWidth, int obsHeight);
        virtual CrossRatioTupleSet determineTupleOfFourPointsForCam(int camIndex);
        virtual ListCrossRatioTupleSet determineTupleOfFourPointsForAllCam();


        
    protected:
        void setTuples(CrossRatioTupleSet tupleSet);
        void setTuplesPerCam(ListCrossRatioTupleSet tupleSetPerCam);
        void setTuplesPerCam(CrossRatioTupleSet tupleSet, int camIndex);

    private:

        CrossRatioTupleSet createsTuples(IntArrayList &combos, IntList &pointSet, GLMList2DVec &points2D);
        CVList2DVec createLinesFromPoints(int imgHeight, int imgWidth, GLMList2DVec &points2D);
        IntArrayList generateCorrespondances(CVList2DVec &lines, GLMList2DVec &points2D);
        void createFeatureImage(cv::Mat &logicalImg, GLMList2DVec &points2D);

        CrossRatioTupleSet collapseListSet(ListCrossRatioTupleSet &tupleSetPerCam);


        GLMListArray2DVec camObservations;
        int obsHeight, obsWidth;

        ListCrossRatioTupleSet tupleSetPerCam;
        CrossRatioTupleSet tupleSet;

    };

    typedef CRTuplesGenerator * CRTuplesGeneratorPtr;
    

} // namespace meshac

#endif // MESH_ACCURACY_CR_TUPLES_GENERATOR_H
