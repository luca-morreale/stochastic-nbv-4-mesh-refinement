
#include <PointKDTree.hpp>
#include <OpenMvgParser.hpp>

#include <omp.h>

#include <glm/glm.hpp>

#include <vector>
#include <array>


#define OMP_THREADS 8

#define ARGS 2

using namespace std;

#pragma omp declare reduction(vec_plus : std::vector<int> : \
                std::transform(omp_out.begin(), omp_out.end(), omp_in.begin(), omp_out.begin(), std::plus<int>())) \
                initializer(omp_priv = omp_orig)


vector<pair<string, string>> mapIntersectionOutput;


int main(int argc, char **argv) {

    omp_set_num_threads(OMP_THREADS);

    if (argc < ARGS + 1) {
        std::cout << "Usage: " << argv[0] << " files_list.txt gt.asc" << std::endl;
        return 1;
    }

    std::string files_list = argv[1];
    std::string gt = argv[2];

    cameval::PointKDTree tree(gt);


    std::ifstream cin(files_list);

    string original;
    string cloud, output;
    int counts;

    while (!cin.eof()) {
        cin >> counts >> original;

        if (counts == 0 || original.empty()) continue;

        for (int i = 0; i < counts; i++) {
            cin >> cloud >> output;
            mapIntersectionOutput.push_back(make_pair(cloud, output));
        }
    }
    cin.close();


    std::cout << mapIntersectionOutput.size() << std::endl;

    int i = 0;
    for (auto el : mapIntersectionOutput) {
        
        OpenMvgParser parser(el.first);
        parser.parse();
        auto points = parser.getSfmData().points_;

        vector<int> counts(10);

        #pragma omp parallel for reduction(vec_plus:counts)
        for (int j = 0; j < points.size(); j++) {

            auto closest = tree.searchClosestPoint(points[j]);
            float distance = glm::distance(closest, points[j]);

            #pragma omp parallel for 
            for (int d = 0; d < 10; d++) {
                if (distance < (float)(d+1)) {
                    counts[d]+=1;
                }
            }
        }
        std::cout << points.size() << std::endl;
        
        std::ofstream out(el.second + "_coverage.txt");
        for (int d = 0; d < 10; d++) {
            double c = counts[d];
            double p = points.size();
            double res = c / p;
            out << (d+1) << " " << res << std::endl;
        }
        out.close();
    }


    return 0;
}
