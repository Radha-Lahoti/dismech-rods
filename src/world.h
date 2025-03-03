#ifndef WORLD_H
#define WORLD_H

#include "global_definitions.h"
// #include "eigenIncludes.h"
// #include "robotDescription.h"


#include "mechanics/soft_robots.h"
#include "mechanics/force_container.h"

// include inner force classes
#include "mechanics/inner_forces/inertial_force.h"
#include "mechanics/inner_forces/elastic_stretching_force.h"
#include "mechanics/inner_forces/elastic_bending_force.h"
#include "mechanics/inner_forces/elastic_twisting_force.h"

// include shell elastic forces
#include "mechanics/inner_forces/elasticStretchingForceShell.h"
#include "mechanics/inner_forces/elasticBendingForceShell.h"

// include time stepper
#include "time_steppers/forward_euler.h"
#include "time_steppers/verlet_position.h"
#include "time_steppers/backward_euler.h"
#include "time_steppers/implicit_midpoint.h"

class SoftRobots;
class ForceContainer;
class BaseTimeStepper;

class World
{
  public:
    World(const std::shared_ptr<SoftRobots>& soft_robots,
          const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params);
    ~World();
    void updateTimeStep();
    double getCoordinate(int i, int limb_idx);
    double getShellCoordinate(int i, int shell_limb_idx);
     Eigen::VectorXd getM1(int i, int limb_idx);
     Eigen::VectorXd getM2(int i, int limb_idx);

    double getCurrentTime() const;
    bool simulationRunning() const;
    int getTimeStep() const;
    void printSimData();
    bool floorExists();
    double getFloorZ();

    std::shared_ptr<SoftRobots> soft_robots;


private:
    std::shared_ptr<ForceContainer> forces;
    std::shared_ptr<BaseTimeStepper> stepper;
    

    int time_step;
    double curr_time;
    double total_time;

    void updateCons();
};

#endif
