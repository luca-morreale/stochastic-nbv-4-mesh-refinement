
#include <omp.h>

#include <glm/glm.hpp>

#include <vector>
#include <regex>

#include <PlyRefiner.hpp>

#define OMP_THREADS 8

#define ARGS 2

using namespace std;


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " in out" << std::endl;
        return 1;
    }
    std::string in = argv[1];
    std::string out = argv[2];

    glm::vec3 basic_color(255.0f, 255.0f, 255.0f);

    cameval::PlyRefiner refiner(in, basic_color);
    refiner.write(out, false);
    

    return 0;
}
