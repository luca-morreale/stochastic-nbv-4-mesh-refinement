#ifndef EVALUATION_CAMERA_POSITION_EVALUATOR_SEQUENCE_H_
#define EVALUATION_CAMERA_POSITION_EVALUATOR_SEQUENCE_H_

#include <EvaluatorDatabase.hpp>

namespace cameval {

    class EvaluatorSequence : public EvaluatorDatabase {
    public:
        EvaluatorSequence(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                            std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile, std::string &sshconfig);
        ~EvaluatorSequence();

    protected:
        virtual std::string getImage(IntStringPair &entry);

    private:
        typedef EvaluatorDatabase super;
    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_EVALUATOR_SEQUENCE_H_
