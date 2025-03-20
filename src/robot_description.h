#ifndef ROBOT_DESCRIPTION_H
#define ROBOT_DESCRIPTION_H

#include "mechanics/force_container.h"
#include "mechanics/soft_robots.h"
#include "global_definitions.h"

// include external forces
#include "mechanics/external_forces/contact_force.h"
#include "mechanics/external_forces/damping_force.h"
#include "mechanics/external_forces/floor_contact_force.h"
#include "mechanics/external_forces/gravity_force.h"
#include "mechanics/external_forces/uniform_constant_force.h"

// include controllers
#include "controllers/active_entanglement_controller.h"
#include "controllers/open_loop_uniform_kappa_bar_controller.h"

// different type of loggers
#include "logging/position_logger.h"
#include "logging/velocity_logger.h"
#include "logging/shell_node_logger.h"

#include "utils/utils.h"


void getRobotDescription(int argc, char** argv, const std::shared_ptr<SoftRobots>& soft_robots,
                         const std::shared_ptr<ForceContainer>& forces,
                         std::shared_ptr<BaseLogger>& logger, SimParams& sim_params,
                         RenderParams& render_params);

void inputProcessorFunc(const std::string inputFileName, std::vector<Eigen::Vector3d>& Nodes, std::vector<std::vector<int>>& facenodes);

#endif
