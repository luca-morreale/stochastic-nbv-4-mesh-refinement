#ifndef EVALUATION_CAMERA_POSITION_OPENMVG_SYSTEM_CALL_H_
#define EVALUATION_CAMERA_POSITION_OPENMVG_SYSTEM_CALL_H_

#include <cstdlib>

#include <utilities.hpp>

namespace cameval {

    class OpenMvgSysCall {
    public:
        static std::string initOpenMvg();
        static std::string imageListing();
        static void extractMvgFeatures(std::string &jsonFile, std::string &pairFile);
        static std::string computeStructureFromPoses(std::string &jsonFile, size_t uniqueId);

        static std::string baseImageFolder;
        static std::string intrinsicParams;

    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_OPENMVG_SYSTEM_CALL_H_
