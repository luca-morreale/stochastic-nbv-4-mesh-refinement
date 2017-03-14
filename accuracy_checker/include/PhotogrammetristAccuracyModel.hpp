
#ifndef MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H
#define MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

#include <boost/bind.hpp>

#include <AccuracyModel.hpp>
#include <alias_definition.hpp>
#include <CRTuplesGenerator.hpp>
#include <ImagePointVarianceEstimator.hpp>
#include <InvalidUpdateException.hpp>

namespace meshac {
    
    class PhotogrammetristAccuracyModel : public AccuracyModel {
    public:
        PhotogrammetristAccuracyModel(GLMList3DVec points3D, CameraMatrixList cameras, 
                        GLMListArray2DVec camObservations, ListMappingGLM2DVec point3DTo2DThroughCam, int obsWidth, int obsHeight);
        
        PhotogrammetristAccuracyModel(GLMList3DVec points3D, CameraList cameras, 
                        GLMListArray2DVec camObservations, ListMappingGLM2DVec point3DTo2DThroughCam, int obsWidth, int obsHeight);
        
        PhotogrammetristAccuracyModel(SfMData data);
        
        ~PhotogrammetristAccuracyModel();
        
        
        /*
         * Computes the matrix of uncertainty for the specified point in coordinates 3D
         * ARGS:
         * int index3DPoint     index of the 3D point wrt to those int the list of 3D points given.
         *
         * RETURNS:
         * a 3x3 matrix for each correspongind 2D point representative of the uncertainty.
         */
        virtual EigMatrixList getAccuracyForPoint(int index3DPoint);


        GLMList3DVec getPoints3D();
        CameraMatrixList getCamerasMatrix();
        GLMListArray2DVec getCamObservations();
        ListMappingGLM2DVec getMapping3DTo2DThroughCam();
        std::pair<int, int> getObservationSize();

        void append3DPoint(GLMVec3 point3D);
        void appendCamera(CameraMatrix cam);

        void setCameraObservations(IntList camIndexs, GLMListArray2DVec newCamObservations);
        void updateCameraObservations(IntList camIndexs, GLMListArray2DVec newCamObservations);
        void updateMapping3DTo2DThroughCam(IntList index3DPoints, ListMappingGLM2DVec indexCams);
        void setMapping3DTo2DThroughCam(IntList index3DPoints, ListMappingGLM2DVec indexCams);


    protected:
        /*
         * Initializes all members.
         */
        virtual void initMembers();

        /*
         * Computes the required cross ratio's tuples.
         */
        virtual void computeTuples();
        virtual void computeTuples(int camIndex);

        /*
         * Evaluates the photogrammetrist's function in the given point with the given camera.
         */
        virtual EigVector4 evaluateFunctionIn(CameraMatrix &cam, GLMVec2 &point);

        /*
         * 
         * 
         */
        virtual EigMatrix computeJacobian(CameraMatrix &cam, GLMVec2 &point);

        /*
         * Getter and Setter for CrossRatio Tuples' Generator.
         */
        void setTupleGenerator(CRTuplesGeneratorPtr generator);
        CRTuplesGeneratorPtr getTuplesGenerator();



        typedef boost::_bi::bind_t<void, boost::_mfi::mf2<void, meshac::CRTuplesGenerator, std::vector<glm::tvec2<float, (glm::precision)0u> >, int>, boost::_bi::list3<boost::_bi::value<meshac::CRTuplesGenerator*>, boost::arg<1>, boost::arg<2> > > FunctionTarget;

        virtual void camObservationGeneralUpdate(IntList &indexs, GLMListArray2DVec &list, GLMListArray2DVec &targetList, FunctionTarget f, std::string errorMsg);
        virtual void mappingGeneralUpdate(IntList &indexs, ListMappingGLM2DVec &list, ListMappingGLM2DVec &targetList, std::string errorMsg);


        /*
         * 
         */
        void setCameras(CameraMatrixList cameras);
        void setCamObservations(GLMListArray2DVec camObservations);
        void setVisibilityOfPoints(ListMappingGLM2DVec point3DTo2DThroughCam);

        float getXh();
        float getYh();

    private:
        CameraMatrixList extractCameraMatrix(CameraList &cameras);

        ImagePointVarianceEstimatorPtr varianceEstimator;
        CRTuplesGeneratorPtr tuplesGenerator;
        CrossRatioTupleSet crossratioTupleSet;


        GLMList3DVec points3D;      // NOT REALLY NEEDED
        CameraMatrixList cameras;
        GLMListArray2DVec camObservations;
        ListMappingGLM2DVec point3DTo2DThroughCam;

        int obsWidth;
        int obsHeight;

        const float xh = 0.01;
        const float yh = 0.01;

    };


} // namespace meshac


#endif // MESH_ACCURACY_PHOTOGRAMMETRIST_ACCURACY_MODEL_H

