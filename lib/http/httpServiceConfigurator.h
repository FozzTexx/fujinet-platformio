/* FujiNet web server helper class

Broke out system configuration functions to make things easier to read.
*/
#ifndef HTTPSERVICECONFIGURATOR_H
#define HTTPSERVICECONFIGURATOR_H

#include <stddef.h>
#include <map>

class fnHttpServiceConfigurator
{
    static void config_printer(std::string printernumber, std::string printermodel, std::string printerport);
public:
    static char * url_decode(char * dst, const char * src, size_t dstsize);
    static std::map<std::string, std::string> parse_postdata(const char * postdata, size_t postlen);
    static int process_config_post(const char * postdata, size_t postlen);
};

#endif // HTTPSERVICECONFIGURATOR_H
