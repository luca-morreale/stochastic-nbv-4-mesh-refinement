#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_DATABASE_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_DATABASE_H_

#include <BasicEvaluator.hpp>
#include <SshHandler.hpp>
#include <Mapper.hpp>

namespace cameval {

    #define SLEEP_SSH_ERROR 10000000    // 10 sec
    #define THRESHOLD_SSH 120000000    // 120 sec
    
    class EvaluatorDatabase : public BasicEvaluator {
    public:
        EvaluatorDatabase(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                            std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile, std::string &sshconfig);
        ~EvaluatorDatabase();

        /***   Evaluate database ***/
        void evaluateDatabase();
        double evaluatePose(IntStringPair &entry, std::string &basicFolder);


    protected:
        virtual void setImagesPattern();

        virtual int getIndexOfSmallestDistance(DoubleList &distances);


        virtual std::string getImage(std::string &imageName);
        virtual Pose getPose(std::string &imageName);


        virtual void setPosesToEvaluate(std::string posesFilename);
        virtual void remapListToQueue(StringList &dataList);
        virtual void appendToPoses(IntStringPair &pair);
    

        virtual std::string initOpenMvg();
        virtual void generateBasicPairFile();


    private:
        std::string posesFilename;
        StringList posesList;
        QueueIntStringPair poses;

        std::string filePrefix;
        std::string fileExtention;

        SshHandlerPtr sshHandler;

        const std::chrono::milliseconds thresholdWaitSsh;

        typedef BasicEvaluator super;
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_DATABASE_H_
