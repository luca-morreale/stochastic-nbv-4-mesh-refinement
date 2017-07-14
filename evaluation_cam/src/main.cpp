
#include <omp.h>

#include <glm/glm.hpp>

#include <Evaluator.hpp>

#define OMP_THREADS 4

#define ARGS 5

int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " points_file.txt ground_truth.asc database.txt sshconfig.txt basicposecam.txt" << std::endl;
        return 1;
    }

    std::string pointsFilename = argv[1];
    std::string groundTruthFilename = argv[2];
    std::string databaseFilename = argv[3];
    std::string sshconfig = argv[4];
    std::string basicPosesFilename = argv[5];

    std::string intrisicParamsFilename = "K.txt";
    std::string baseImageFolder = "images";

    cameval::Evaluator eval(pointsFilename, groundTruthFilename, databaseFilename, basicPosesFilename, baseImageFolder, intrisicParamsFilename, sshconfig);
    eval.evaluate();

    return 0;
}


