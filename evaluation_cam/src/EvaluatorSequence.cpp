#include <EvaluatorSequence.hpp>

namespace cameval {

    EvaluatorSequence::EvaluatorSequence(std::string &databaseFilename, std::string &groundTruthFilename, std::string &basicPoseFilename, 
                            std::string &baseImageFolder, std::string &intrinsicParams, std::string &outputFile, std::string &sshconfig)
    : EvaluatorDatabase(databaseFilename, groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, outputFile, sshconfig)

    { /*    */ }

    EvaluatorSequence::~EvaluatorSequence()
    { /*    */ }


    std::string EvaluatorSequence::getImage(IntStringPair &entry)
    {
        std::string filename = getMapper()->mapStringToFile(entry.second);
        IntStringPair pair(entry.first, filename);

        return super::getImage(pair);
    }



} // namespace cameval
