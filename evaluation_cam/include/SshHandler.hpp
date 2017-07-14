#ifndef EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_
#define EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_

#include <iostream>
#include <fstream>

#include <libssh/libssh.h>

#include <type_definition.h>
#include <SSHException.hpp>

#define LIBSSH_STATIC 1
#define BUFFER_SIZE 16384

namespace cameval {
    
    class SshHandler {
    public:
        SshHandler(std::string &host, std::string &user, std::string &password);
        SshHandler(std::string &sshconfig);
        ~SshHandler();

        std::string download(std::string &filename);

    protected:
        virtual void setOptions(ssh_session session);
        virtual void throwException(std::string prefix, std::string error, ssh_session session);
        virtual void throwException(std::string prefix, std::string error, ssh_scp scp, ssh_session session);

    private:
        std::string host;
        std::string user;
        std::string password;


    };

    typedef SshHandler* SshHandlerPtr;

} // namespace cameval

#endif // EVALUATION_CAMERA_POSITION_SSH_HANDLER_H_
