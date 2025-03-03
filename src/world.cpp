#include "world.h"
#include "mechanics/elastic_joint.h"
#include "mechanics/elastic_rod.h"
#include "mechanics/external_forces/contact_force.h"
#include "mechanics/external_forces/floor_contact_force.h"
#include "mechanics/force_container.h"
#include "mechanics/inner_forces/elastic_bending_force.h"
#include "mechanics/inner_forces/elastic_stretching_force.h"
#include "mechanics/inner_forces/elastic_twisting_force.h"
#include "mechanics/inner_forces/inertial_force.h"
#include "mechanics/inner_forces/elasticStretchingForceShell.h"
#include "mechanics/inner_forces/elasticBendingForceShell.h"
#include "mechanics/soft_robots.h"
#include "time_steppers/backward_euler.h"
#include "time_steppers/forward_euler.h"
#include "time_steppers/implicit_midpoint.h"
#include "time_steppers/verlet_position.h"

World::World(const std::shared_ptr<SoftRobots>& soft_robots,
             const std::shared_ptr<ForceContainer>& forces, const SimParams& sim_params)
    : soft_robots(soft_robots), forces(forces), time_step(0), curr_time(0.0),
      total_time(sim_params.sim_time) {

    
    // Declare inner elastic forces. 
    // add shell elastic forces
        forces->addForce(std::make_shared<elasticStretchingForceShell>(soft_robots));
        forces->addForce(std::make_shared<elasticBendingForceShell>(soft_robots));

        forces->addForce(std::make_shared<ElasticStretchingForce>(soft_robots));
        forces->addForce(std::make_shared<ElasticBendingForce>(soft_robots));
        forces->addForce(std::make_shared<ElasticTwistingForce>(soft_robots));
    
    // std::cout<< "no. of forces after adding elastic forces: " << forces->forces.size()<< std::endl;

    // Declare inertial force. Should be avoided for explicit methods
    if (sim_params.integrator != FORWARD_EULER && sim_params.integrator != VERLET_POSITION) {
        forces->addForce(std::make_shared<InertialForce>(soft_robots));
    }

    // std::cout<< "no. of forces after adding inertial force: " << forces->forces.size()<< std::endl;

    // Set up the time stepper
    switch (sim_params.integrator) {
        case FORWARD_EULER:
            stepper = std::make_shared<ForwardEuler>(soft_robots, forces, sim_params);
            break;
        case VERLET_POSITION:
            stepper = std::make_shared<VerletPosition>(soft_robots, forces, sim_params);
            break;
        case BACKWARD_EULER:
            std::cout<<"selected backwardEuler"<< std::endl;
            stepper = std::make_shared<BackwardEuler>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            std::cout<<"constructed backwardEuler"<< std::endl;
            break;
        case IMPLICIT_MIDPOINT:
            stepper =
                std::make_shared<ImplicitMidpoint>(soft_robots, forces, sim_params, PARDISO_SOLVER);
            break;
    }
    std::cout<<"after stepper declaration"<< std::endl;

    stepper->initStepper();

    std::cout<<"after the initStepper() step"<< std::endl;

    
    // for (const auto& limb : soft_robots->limbs) std::cout<<"rod-limb no. of vertices: "<<limb->nv; // this kind of thing doesn't give errors

    // std::cout<< soft_robots->limbs[0]->nv; //issue

    // std::cout<<"size of offsets for rod limbs set for the time-stepper linked with force0:"<< forces->forces[0]->stepper->offsets.size()<< std::endl; gives segmentation fault

    if (sim_params.enable_2d_sim) {
        for (const auto& limb : soft_robots->limbs)
            limb->enable2DSim();
    }

    // Update boundary conditions
    updateCons();

    std::cout<<"updateCons() done"<< std::endl;

    // Allocate every thing to prepare for the first iteration
    stepper->updateSystemForNextTimeStep();

    std::cout<<"stepper ->updateSystemForNextTimeStep() done"<< std::endl;
}

World::~World() = default;

void World::updateCons() {
    for (const auto& limb : soft_robots->limbs)
        limb->updateMap();
    for (const auto &shell_limb : soft_robots->shell_limbs)
        shell_limb->updateMap();

    std::cout<<"we are here"<< std::endl;
    stepper->update();
}

void World::updateTimeStep() {
    curr_time += stepper->stepForwardInTime();
    time_step++;
}

void World::printSimData() {
    auto cf = forces->cf;
    auto ff = forces->ff;
    if (cf && ff) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f | "
                   "floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, cf->getNumCollisions(), cf->getMinDist(),
                   ff->num_contacts, ff->min_dist);
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s | "
                   "floor_con: %i | f_min_dist: %.6f\n",
                   curr_time, stepper->iter, 0, "N/A", ff->num_contacts, ff->min_dist);
        }
    }
    else if (cf) {
        if (cf->getNumCollisions() > 0) {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %.6f\n", curr_time, stepper->iter,
                   cf->getNumCollisions(), cf->getMinDist());
        }
        else {
            printf("time: %.4f | iters: %i | con: %i | min_dist: %s\n", curr_time, stepper->iter, 0,
                   "N/A");
        }
    }
    else if (ff) {
        printf("time: %.4f | iters: %i | floor_con: %i | f_min_dist: %.6f\n", curr_time,
               stepper->iter, ff->num_contacts, ff->min_dist);
    }
    else {
        printf("time: %.4f | iters: %i\n", curr_time, stepper->iter);
    }
}

bool World::simulationRunning() const {
    if (curr_time < total_time)
        return true;
    else {
        std::cout << "Completed simulation." << std::endl;
        return false;
    }
}

double World::getCoordinate(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->x[i];
}

double World::getShellCoordinate(int i, int shell_limb_idx)
{
    return soft_robots->shell_limbs[shell_limb_idx]->x[i];
}


Eigen::VectorXd World::getM1(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->m1.row(i);
}

Eigen::VectorXd World::getM2(int i, int limb_idx) {
    return soft_robots->limbs[limb_idx]->m2.row(i);
}

int World::getTimeStep() const {
    return time_step;
}

double World::getCurrentTime() const {
    return curr_time;
}

bool World::floorExists() {
    return forces->ff != nullptr;
}

double World::getFloorZ() {
    if (forces->ff)
        return forces->ff->floor_z;
    throw std::runtime_error("Floor does not exist.");
}
