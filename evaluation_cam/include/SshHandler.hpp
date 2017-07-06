#ifndef EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_
#define EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_

#include <iostream>
#include <fstream>

#include <libssh/libssh.h>

#include <type_definition.h>
#include <SSHException.hpp>

#define LIBSSH_STATIC 1

namespace cameval {
    
    class SshHandler {
    public:
        SshHandler(std::string &host, std::string &user, std::string &password);
        SshHandler(std::string &sshconfig);
        ~SshHandler();

        void download(std::string &filename);

    protected:
        virtual int fetchFile(ssh_session session, std::string &filePath);

    private:
        std::string host;
        std::string user;
        std::string password;

    };

    typedef SshHandler* SshHandlerPtr;

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_
