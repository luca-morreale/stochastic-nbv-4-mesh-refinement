#ifndef EVALUATION_CAMERA_POSITION_SEQUENTIAL_POSE_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_SEQUENTIAL_POSE_EVALUATOR_H_

#include <regex>

#include <boost/filesystem.hpp>

#include <aliases.h>
#include <PointCloudIntersecter.hpp>
#include <PoseEvaluator.hpp>
#include <utilities.hpp>

namespace cameval {

    class SequentialEvaluator : public PoseEvaluator {
    public:
        SequentialEvaluator(std::string &pointsFile, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                                        std::string &baseImageFolder, std::string &intrinsicParams, 
                                        std::string &reportFile, std::string &basicPovFile);
        ~SequentialEvaluator();

        virtual void multistepEvaluation(std::string reconstructionExe, std::string accuracyExe, std::string optimalCamExe);

    protected:
        virtual void cleanFiles(StringList files) override;
        virtual std::string transformToLookat(std::string &fixedPosefile);
        

        virtual double evaluateIntersectedPose(IntStringPair &entry, std::string &basicFolder);

        int execute(std::string command);
    
    private:
        int steps;
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_SEQUENTIAL_POSE_EVALUATOR_H_
