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

    void SshHandler::setOptions(ssh_session session)
    {
        ssh_options_set(session, SSH_OPTIONS_HOST, this->host.c_str());
        ssh_options_set(session, SSH_OPTIONS_USER, this->user.c_str());
    }

    void SshHandler::throwException(std::string prefix, std::string error, ssh_session session)
    {
        ssh_free(session);
        throw SSHException(prefix + error + "\n");
    }

    void SshHandler::throwException(std::string prefix, std::string error, ssh_scp scp, ssh_session session)
    {
        ssh_scp_free(scp);
        ssh_free(session);
        throw SSHException(prefix + error + "\n");
    }


    std::string SshHandler::download(std::string &filepath)
    {
        int rc;
        int size, mode;
        char *filename;
        char buffer[BUFFER_SIZE];

        ssh_session session = ssh_new();
        if (session == NULL) {
            throw SSHException("Unable to create a session\n");
        }
        
        setOptions(session);

        rc = ssh_connect(session);
        if (rc != SSH_OK) {
            throwException("Error connecting to localhost: ", ssh_get_error(session), session);
        }

        rc = ssh_userauth_password(session, NULL, this->password.c_str());
        if (rc != SSH_AUTH_SUCCESS) {
            throwException("Error authenticating with password: ", ssh_get_error(session), session);
        }

        ssh_scp scp = ssh_scp_new(session, SSH_SCP_READ, filepath.c_str());
        if (scp == NULL) {
            throwException("Error allocating scp session: ", ssh_get_error(session), session);
        }

        if(ssh_scp_init(scp) != SSH_OK){
            throwException("Error allocating scp session: ", ssh_get_error(session), scp, session);
        }

        rc = ssh_scp_pull_request(scp);
        if (rc != SSH_SCP_REQUEST_NEWFILE) {
            throwException("Error receiving information about file: ", ssh_get_error(session), scp, session);
        }

        size = ssh_scp_request_get_size(scp);
        filename = strdup(ssh_scp_request_get_filename(scp));
        mode = ssh_scp_request_get_permissions(scp);

        std::string localFilename(filename);
        free(filename);

        
        if (buffer == NULL) {
            throw SSHException("Memory allocation error\n");
        }

        unsigned int sizeAcc = 0;
        std::ofstream out(localFilename);
        ssh_scp_accept_request(scp);
        int read_data = 0;

        do {
            int missing_size = (sizeof(buffer) < size - sizeAcc) ? sizeof(buffer) : size - sizeAcc;

            read_data = ssh_scp_read(scp, buffer, missing_size);
            if (read_data == SSH_ERROR) {
                throwException("Error receiving file data: ", ssh_get_error(session), scp, session);
            }
            out.write(buffer, read_data);
            
            sizeAcc += read_data;
        } while (sizeAcc != size);

        out.close();

        rc = ssh_scp_pull_request(scp);
        if (rc != SSH_SCP_REQUEST_EOF) {
            throwException("Unexpected request: ", ssh_get_error(session), scp, session);
        }

        ssh_scp_close(scp);
        ssh_scp_free(scp);

        return localFilename;
    }


} // namespace cameval
