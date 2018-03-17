#include "logger.h"

Logger::Logger(){
    log_path = "./";
    file_name = "log.csv"; 
    ifstream filecheck(file_name);

    if( !filecheck.good() ){
        ofstream outfile_(log_path + file_name);
        outfile_.close();
    }

    outfile.open(log_path + file_name, ios_base::app);
}

Logger::~Logger(){}

void Logger::log(const string & line){
    outfile << line.c_str() << std::endl;
}

