
#include <omp.h>

#include <glm/glm.hpp>


#include <Evaluator.hpp>

#define OMP_THREADS 1

#define ARGS 4

#define DEBUG 1

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " database.txt ground_truth.asc sshconfig.txt basicposecam.txt" << std::endl;
        return 1;
    }

    std::string databaseFilename = argv[1];
    std::string groundTruthFilename = argv[2];
    std::string sshconfig = argv[3];
    std::string basicPosesFilename = argv[4];

    std::string intrisicParamsFilename = "K.txt";
    std::string baseImageFolder = "images";

    std::ifstream cin(intrisicParamsFilename);
    std::string intrinsicParams;
    cin >> intrinsicParams;
    cin.close();

    // databaseFilename, groundTruthFilename, basicPoseFilename, baseImageFolder, intrinsicParams, sshconfig

    cameval::Evaluator eval(databaseFilename, groundTruthFilename, basicPosesFilename, baseImageFolder, intrinsicParams, sshconfig);
    eval.evaluate();

    return 0;
}



