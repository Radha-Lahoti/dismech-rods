#ifndef SHELLNODE_LOGGER_H
#define SHELLNODE_LOGGER_H

#include "base_logger.h"
#include "global_definitions.h"


class ShellNodeLogger : public BaseLogger {
public:
    ShellNodeLogger(std::string logfile_base, std::ofstream& df, int per);
    ShellNodeLogger(std::string logfile_base, std::string logfile_suffix, std::ofstream& df, int per);
    ~ShellNodeLogger();
private:
    std::string getLogHeader() override;
    std::string getLogData() override;
};


#endif
