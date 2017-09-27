#ifndef EVALUATION_CAMERA_POSITION_SYSTEM_EVALUATOR_H_
#define EVALUATION_CAMERA_POSITION_SYSTEM_EVALUATOR_H_

#include <regex>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <aliases.h>
#include <BasicEvaluator.hpp>
#include <utilities.hpp>

namespace cameval {

    class SystemEvaluator : public BasicEvaluator {
    public:
        SystemEvaluator(std::string &groundTruthFilename, std::string &basicPoseFilename, std::string &baseImageFolder, 
            std::string &intrinsicParams, std::string &reportFile, std::string &basicPovFile);
        ~SystemEvaluator();

        
        void systemEvaluation(int steps, std::string reconstructionExe, std::string accuracyExe, std::string optimalCamExe);

    protected:
        virtual Pose getPose(std::string &data);
        virtual std::string getImage(std::string &poseString);
        virtual void cleanFiles(StringList files) override;

        virtual void generatePovrayFile(std::string &file, std::string &poseString);
        virtual std::string generatePovrayDeclaration(std::string &data);

        virtual std::string initOpenMvg();
        std::string generatePairFile(size_t uniqueId);
        
        std::string getTimeStamp();

        virtual std::string transformToLookat(std::string &fixedPosefile);  // READ just one pose!

        virtual void cleanFeatures();
        virtual void cleanMatches();

    private:
        std::string basicPovFile;

        typedef BasicEvaluator super;
        
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_SYSTEM_EVALUATOR_H_
