#ifndef LOGGER_H_
#define LOGGER_H_
#include <string>
#include <fstream>
using namespace std;

class Logger {
public:
    Logger();
    virtual ~Logger();
    void log(const string &line);
    void close();

private:
    ofstream outfile;
    string log_path;
    string file_name;

};

#endif  /* LOGGER_H_  */