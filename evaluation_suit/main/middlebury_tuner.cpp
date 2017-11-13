#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>

#include <utilities.hpp>
#include <aliases.h>
#include <MiddleburyDatasetEvaluator.hpp>

#define ARGS 5
#define MAX_IT 5

int main(int argc, char **argv) {

    double init = 0.1;
    double step = 0.5;
    double vm = 0, v = 0, p = 0, c = 0;
    std::string command;

    for (int i = 1; i < MAX_IT; i++) {
        for (int j = 1; j < MAX_IT; j++) {
            for (int k = 3; k < MAX_IT; k++) {
                for (int t = 3; t < MAX_IT; t++) {
                    // if (t ==0 && k==0 &&j==0 && i==0) t+=1;
                    vm = init + i * step;
                    v = init + j * step;
                    p = init + k * step;
                    c = init + t * step;

                    std::ofstream cout("scorer_params.txt");
                    cout << vm << std::endl << v << std::endl << p << std::endl << c << std::endl;
                    cout.close();

                    std::string prefix = std::to_string(vm) + "_" + std::to_string(v) + "_" + std::to_string(p) + "_" + std::to_string(c);
                    command = "sh exec_all.sh " + prefix;
                    system(command.c_str());
                }
            }
        }
    }

    return 0;
}
