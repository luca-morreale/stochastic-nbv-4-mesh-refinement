#ifndef EVALUATION_CAMERA_POSITION_POSE_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_POSE_EVALUATOR_H_

#include <regex>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <aliases.h>
#include <BasicEvaluator.hpp>
#include <utilities.hpp>

namespace cameval {

    class PoseEvaluator : public BasicEvaluator {
    public:
        PoseEvaluator(std::string &pointsFile, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, 
                                        std::string &reportFile, std::string &basicPovFile);
        ~PoseEvaluator();

    protected:
        virtual std::string getImage(IntStringPair &entry);
        virtual std::string generatePovrayDeclaration(std::string &data);
        virtual Pose getPose(std::string &data);
        virtual std::string getTimeStamp();

    private:
        std::string basicPovFile;
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_H_
