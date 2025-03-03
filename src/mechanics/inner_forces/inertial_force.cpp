#include "inertial_force.h"
#include "mechanics/elastic_joint.h"
#include "mechanics/elastic_rod.h"
#include "mechanics/soft_robots.h"
#include "time_steppers/base_time_stepper.h"

InertialForce::InertialForce(const std::shared_ptr<SoftRobots>& m_soft_robots)
    : BaseForce(m_soft_robots) {
}

InertialForce::~InertialForce() {
    ;
}

void InertialForce::computeForce(double dt) {
    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i])
                continue;
            f = (limb->mass_array[i] / dt) * ((limb->x[i] - limb->x0[i]) / dt - limb->u[i]);
            stepper->addForce(i, f, limb_idx);
        }
        limb_idx++;
    }

    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            f = (joint->mass / dt) * ((joint->x[i] - joint->x0[i]) / dt - joint->u[i]);
            stepper->addForce(4 * joint->joint_node + i, f, joint->joint_limb);
        }
    }

    // shell
    int shell_limb_idx = 0;
    for (const auto& shell_limb : soft_robots->shell_limbs) {
        for (int i=0; i < shell_limb->ndof; i++)
        {
            // if (shell_limb->isDOFJoint[i]) continue;
            f = (shell_limb->mass_array[i] / dt) * ((shell_limb->x[i] - shell_limb->x0[i]) / dt - shell_limb->u[i]);
            stepper->addForceShell(i, f, shell_limb_idx);
        }
        shell_limb_idx++;
    }

}

void InertialForce::computeForceAndJacobian(double dt) {
    computeForce(dt);

    int limb_idx = 0;
    for (const auto& limb : soft_robots->limbs) {
        for (int i = 0; i < limb->ndof; i++) {
            if (limb->isDOFJoint[i])
                continue;
            jac = limb->mass_array(i) / (dt * dt);
            stepper->addJacobian(i, i, jac, limb_idx);
        }
        limb_idx++;
    }

    int ind;
    for (const auto& joint : soft_robots->joints) {
        for (int i = 0; i < 3; i++) {
            jac = joint->mass / (dt * dt);
            ind = 4 * joint->joint_node;
            stepper->addJacobian(ind + i, ind + i, jac, joint->joint_limb);
        }
    }

    // shell
    int shell_limb_idx = 0;
    for (const auto& shell_limb : soft_robots->shell_limbs) {
        for (int i=0; i < shell_limb->ndof; i++)
        {
            // if (shell_limb->isDOFJoint[i]) continue;
            jac = shell_limb->mass_array(i) / (dt * dt);
            stepper->addJacobianShell(i, i, jac, shell_limb_idx);
        }
        shell_limb_idx++;
    }
}
