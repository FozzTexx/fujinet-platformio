/**
 * FTP class for #FujiNet
 */

#ifndef FNFTP_H
#define FNFTP_H

#include <string>
#include "../tcpip/fnTcpClient.h"

using namespace std;

class fnFTP
{
public:

    /**
     * Log into FTP server.
     * @param hostname The host to connect to.
     * @param port the control port # to connect to. Default is 21.
     * @return TRUE on error, FALSE on success
     */
    bool login(string hostname, unsigned short port = 21);

    /**
     * Log out of FTP server, closes control connection.
     * @return TRUE on error, FALSE on success.
     */
    bool logout();

protected:
private:

    /**
     * The fnTCP client used for control connection
     */
    fnTcpClient control;

    /**
     * The fnTCP client used for data connection
     */
    fnTcpClient data;

    /**
     * last response from control connection.
     */
    string controlResponse;

    /**
     * read and parse control response
     * @return the numeric response
     */
    string get_response();

};

#endif /* FNFTP_H */