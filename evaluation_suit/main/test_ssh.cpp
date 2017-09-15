#include <SshHandler.hpp>


int main(int argc, char **argv) {
    std::string sshconfig = argv[1];


    cameval::SshHandlerPtr sshHandler = new cameval::SshHandler(sshconfig);

    std::string filepath = "/media/storage_big/building/session2/building_-1.2_10.0_-150.0_0.0_0.0_180.0.png";
    std::string file = sshHandler->download(filepath);
    std::cout << file << std::endl;

    delete sshHandler;

    return 0;
}
