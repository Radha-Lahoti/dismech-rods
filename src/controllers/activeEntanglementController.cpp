#include "activeEntanglementController.h"
#include "mechanics/softRobots.h"


activeEntanglementController::activeEntanglementController(const shared_ptr <softRobots> &soft_robots,
                                                           double start_time, double end_time) :
                                                           baseController(soft_robots->limbs),
                                                           start_time(start_time), end_time(end_time)
{
    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];

        random_curvatures.emplace_back();

        for (int i = 1; i < limb->ne; i++) {
            double random_curvature = rand() % 40 * (M_PI / 180);
            random_curvatures[limb_idx].push_back(random_curvature);
        }
    }
}


activeEntanglementController::~activeEntanglementController() = default;


void activeEntanglementController::updateTimeStep(double dt) {
    baseController::updateTimeStep(dt);

    if (current_time < start_time || current_time > end_time) return;

    double curr_ratio = 0.5 * (current_time - start_time) / (end_time - start_time);

    for (int limb_idx = 0; limb_idx < num_actuators; limb_idx++) {
        auto limb = limbs[limb_idx];

        for (int i = 1; i < limb->ne; i++) {
            double random_curvature = 2 * tan(random_curvatures.at(limb_idx).at(i-1) * curr_ratio * 0.5);
            limb->kappa_bar(i, 0) = random_curvature;
            limb->kappa_bar(i, 1) = random_curvature;
        }
    }
}
