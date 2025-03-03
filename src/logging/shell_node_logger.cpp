#include "shell_node_logger.h"

#include <utility>
#include "world.h"


ShellNodeLogger::ShellNodeLogger(std::string logfile_base, std::ofstream &df, int per) :
                             BaseLogger("node", std::move(logfile_base), df, per)
{
}


ShellNodeLogger::ShellNodeLogger(std::string logfile_base, std::string logfile_suffix, std::ofstream &df, int per) :
        BaseLogger("node", std::move(logfile_suffix), std::move(logfile_base), df, per)
{
}


ShellNodeLogger::~ShellNodeLogger() = default;

std::string ShellNodeLogger::getLogHeader() {
    return "";
}

std::string ShellNodeLogger::getLogData() {
    std::ostringstream log_data;
    log_data << world_ptr->getCurrentTime();
    for (const auto& shell_limb : world_ptr->soft_robots->shell_limbs) {
        for (int i = 0; i < shell_limb->nv; i++) {
            Eigen::Vector3d v = shell_limb->getVertex(i);
            log_data << "," << v(0) << "," << v(1) << "," << v(2);
        }
    }
    return log_data.str();
}