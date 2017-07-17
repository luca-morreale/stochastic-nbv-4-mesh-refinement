#ifndef EVALUATION_CAMERA_POSITION_FILE_HANDLER_H_
#define EVALUATION_CAMERA_POSITION_FILE_HANDLER_H_

#include <aliases.h>
#include <SourceDirectoryDoesNotExistsException.hpp>
#include <UnableToCreateDirectoryException.hpp>

namespace cameval {
    
    class FileHandler {
    public:
        static void createDestinationFolder(BoostPath &destination);

        static void copyFile(std::string &sourceFile, std::string &destFile);
        static void copyFile(BoostPath &source, BoostPath &dest);
        static void copyDirectory(std::string &sourceFolder, std::string &destinationFolder);
        static void copyDirectory(BoostPath &source, BoostPath &destination);
        static void copyAllFilesFromTo(BoostPath &source, BoostPath &destination);
        static void copyFileInto(BoostPath &source, BoostPath &dest);
        static void copyFileInto(std::string &sourceString, std::string &destString);
        
        static void moveFileInside(std::string &sourceString, std::string &destString);
        static void moveFileInside(BoostPath &source, BoostPath &dest);

        static void cleanAll(StringList folders);
        
        static bool checkSourceExistence(std::string &sourceFile);
        static bool checkSourceExistence(BoostPath &source);
        static bool isSpaceSystemError(BoostSystemErrorCode errorCode);

    };

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_FILE_HANDLER_H_
