#ifndef ROBOTDESCRIPTION_H
#define ROBOTDESCRIPTION_H

#include <stdexcept>

#include "mechanics/softRobots.h"
#include "mechanics/forceContainer.h"

// include external forces
#include "mechanics/external_forces/dampingForce.h"
#include "mechanics/external_forces/gravityForce.h"
#include "mechanics/external_forces/floorContactForce.h"
#include "mechanics/external_forces/uniformConstantForce.h"
#include "mechanics/external_forces/contactForce.h"

// include controllers
#include "controllers/openLoopUniformKappaBarController.h"
#include "controllers/activeEntanglementController.h"

// different type of loggers
#include "logging/rodNodeLogger.h"
#include "logging/shellNodeLogger.h"
#include "logging/velocityLogger.h"

#include "utils/utils.h"


typedef enum {FORWARD_EULER,
              VERLET_POSITION,
              BACKWARD_EULER,
              IMPLICIT_MIDPOINT}
              numerical_integration_scheme;


struct simParams {
    double sim_time = 10;                              // Total time for simulation [s]
    double dt = 1e-3;                                  // Time step size [s]
    bool render = true;                                // Live OpenGL rendering
    bool show_mat_frames = false;                      // Render material frames
    double render_scale = 1.0;                         // Rendering scale
    double dtol = 1e-2;                                // Dynamics tolerance [m/s]*
    double ftol = 1e-4;                                // Force tolerance*
    int max_iter = 500;                                // Maximum iterations for a time step
    int adaptive_time_stepping = 0;                    // Adaptive time stepping*
    int cmd_line_per = 1;                              // Command line sim info output period
    bool enable_2d_sim = false;                        // Lock z and theta DOFs
    bool line_search = true;                           // Enable line search method
    numerical_integration_scheme nis = BACKWARD_EULER; // Numerical integration scheme*
    int debug_verbosity = 1;                           // Prints certain debug statements
    bool structure_shell = false;                      // True if simulating shell, else false if simulating rod
};


void get_robot_description(int argc, char** argv,
                           const shared_ptr<softRobots>& soft_robots,
                           const shared_ptr<forceContainer>& forces,
                           shared_ptr<worldLogger>& logger,
                           simParams& sim_params);

void inputProcessorFunc(const std::string inputFileName, vector<Vector3d>& Nodes, vector<vector<int>>& facenodes);

#endif
