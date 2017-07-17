#include <InputReader.hpp>

namespace cameval {

    StringList InputReader::readDatabase(std::string &database) 
    {
        std::ifstream cin(database);
        StringList filelist;

        while (!cin.eof()) {
            std::string file;
            cin >> file;
            file = trim(file);
            if (file.size() > 0){
                filelist.push_back(file);
            }
        }
        cin.close();
        return filelist;
    }

} // namespace cameval
