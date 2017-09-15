#include <FileHandler.hpp>

namespace cameval {

    void FileHandler::createDestinationFolder(BoostPath &destination)
    {
        if(boost::filesystem::exists(destination)) {
            std::cerr << "Destination directory " << destination.string() << " already exists." << std::endl;
        } else if(!boost::filesystem::create_directory(destination)) {      // NOTE Create the destination directory
            std::cerr << "Unable to create destination directory" << destination.string() << std::endl;
            throw UnableToCreateDirectoryException(destination);
        }
    }

    void FileHandler::copyFile(std::string &sourceFile, std::string &destFile)
    {
        BoostPath source(sourceFile);
        BoostPath dest(destFile);
        copyFile(source, dest);
    }

    void FileHandler::copyFile(BoostPath &source, BoostPath &dest)
    {
        boost::filesystem::copy_file(source, dest, boost::filesystem::copy_option::overwrite_if_exists);
    }

    void FileHandler::copyDirectory(std::string &sourceFolder, std::string &destinationFolder)
    {
        BoostPath source(sourceFolder);
        BoostPath destination(destinationFolder);
        copyDirectory(source, destination);
    }

    void FileHandler::copyDirectory(BoostPath &source, BoostPath &destination)
    {
        checkSourceExistence(source);
        createDestinationFolder(destination);

        copyAllFilesFromTo(source, destination);
    }

    void FileHandler::copyAllFilesFromTo(BoostPath &source, BoostPath &destination)
    {
        // Iterate through the source directory
        for(boost::filesystem::directory_iterator file(source); file != boost::filesystem::directory_iterator(); file++) {
            BoostPath current(file->path());
            copyFileInto(current, destination);
        }
    }

    void FileHandler::copyFileInto(std::string &sourceString, std::string &destString)
    {
        BoostPath source(sourceString);
        BoostPath dest(destString);
        copyFileInto(source, dest);
    }

    void FileHandler::copyFileInto(BoostPath &source, BoostPath &dest)
    {
        boost::filesystem::copy_file(source, dest / source.filename(), boost::filesystem::copy_option::overwrite_if_exists);
    }

    void FileHandler::moveFileInside(std::string &sourceString, std::string &destString)
    {
        BoostPath dest(destString);
        BoostPath source(sourceString);
        copyFileInto(source, dest);
    }

    void FileHandler::moveFileInside(BoostPath &source, BoostPath &dest)
    {
        boost::filesystem::rename(source, dest / source.filename());
    }

    void FileHandler::cleanAll(StringList folders)
    {
        for (auto folder : folders) {
            BoostPath path(folder);
            boost::filesystem::remove_all(path);
        }
    }

    bool FileHandler::checkSourceExistence(std::string &sourceFile)
    {
        BoostPath source(sourceFile);
        return checkSourceExistence(source);
    }

    bool FileHandler::checkSourceExistence(BoostPath &source)
    {
        if(!boost::filesystem::exists(source) || !boost::filesystem::is_directory(source)) {
            return false;
            // throw SourceDirectoryDoesNotExistsException(source);
        }
        return true;
    }

    bool FileHandler::isSpaceSystemError(BoostSystemErrorCode errorCode)
    {
        return errorCode == boost::system::errc::errc_t::file_too_large ||
                errorCode == boost::system::errc::errc_t::not_enough_memory ||
                errorCode == boost::system::errc::errc_t::no_space_on_device;
    }


} // namespace cameval
