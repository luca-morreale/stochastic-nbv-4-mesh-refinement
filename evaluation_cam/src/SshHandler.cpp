#include <SshHandler.hpp>

namespace cameval {

    SshHandler::SshHandler(std::string &host, std::string &user, std::string &password)
    {
        this->host = host;
        this->user = user;
        this->password = password;
    }

    SshHandler::SshHandler(std::string &sshconfig)
    {
        std::ifstream cin(sshconfig);
        cin >> host;
        cin >> user;
        cin >> password;
        cin.close();
    }

    SshHandler::~SshHandler()
    { /*    */ }

    void SshHandler::download(std::string &filePath) 
    {
        ssh_session session;
        std::string host = this->user + "@" + this->host;
        ssh_options_set(session, SSH_OPTIONS_HOST, host.c_str());

        if(ssh_connect(session) != SSH_OK) {
            std::string err = ssh_get_error(session);
            std::cerr << "Error connecting to localhost: " << err << std::endl;
            ssh_free(session);
            throw SSHException(err);
        }
        
        if (ssh_userauth_password(session, NULL, this->password.c_str()) != SSH_AUTH_SUCCESS) {
            std::string err = ssh_get_error(session);
            std::cerr << "Error authenticating with password: " << err << std::endl;
            ssh_disconnect(session);
            ssh_free(session);
            throw SSHException(err);
        }
      
        fetchFile(session, filePath);
        ssh_disconnect(session);
        ssh_finalize();
    }

    int SshHandler::fetchFile(ssh_session session, std::string &filePath)
    {
        int rc;
        int size, mode;
        char *filename, *buffer;

        ssh_scp scp=ssh_scp_new(session, SSH_SCP_READ | SSH_SCP_RECURSIVE, filePath.c_str());
        
        rc = ssh_scp_pull_request(scp);
        if (rc != SSH_SCP_REQUEST_NEWFILE) {
            std::string err = ssh_get_error(session);
            std::cerr << "Error receiving information about file: " << err << std::endl;
            throw SSHException(err);
        }
        size = ssh_scp_request_get_size(scp);
        filename = strdup(ssh_scp_request_get_filename(scp));
        mode = ssh_scp_request_get_permissions(scp);
        
        free(filename);
        buffer = malloc(size);
        if (buffer == NULL) {
            std::cerr << "Memory allocation error" << std::endl;
            throw SSHException("Memory allocation error");
        }
        ssh_scp_accept_request(scp);
        rc = ssh_scp_read(scp, buffer, size);
        if (rc == SSH_ERROR) {
            std::string err = ssh_get_error(session);
            std::cerr << "Error receiving file data: " << err << std::endl;
            free(buffer);
            throw SSHException(err);
        }
        
        write(1, buffer, size);
        free(buffer);
        rc = ssh_scp_pull_request(scp);
        if (rc != SSH_SCP_REQUEST_EOF) {
            std::string err = ssh_get_error(session);
            std::cerr << "Unexpected request: " << err << std::endl;
            throw SSHException(err);
        }

        return SSH_OK;
    }


    



} // namespace cameval
