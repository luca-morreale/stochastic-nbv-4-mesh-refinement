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

    protected:
        virtual std::string getImage(IntStringPair &entry);
        virtual Pose getPose(std::string &data);

        MapperPtr getMapper();

    private:
        SshHandlerPtr sshHandler;
        MapperPtr mapper;

        const std::chrono::milliseconds thresholdWaitSsh;
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_DATABASE_H_
