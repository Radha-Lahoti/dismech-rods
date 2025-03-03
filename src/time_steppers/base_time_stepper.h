#ifndef BASETIMESTEPPER_H
#define BASETIMESTEPPER_H

#include "global_definitions.h"
#include "mechanics/force_container.h"
#include "robot_description.h"

class SoftRobots;
class ElasticRod;
class ElasticJoint;
class ElasticShell;
class BaseController;


class BaseTimeStepper : public std::enable_shared_from_this<BaseTimeStepper>
{
public:
    BaseTimeStepper(const std::shared_ptr<SoftRobots>& m_soft_robots,
                    const std::shared_ptr<ForceContainer>& m_forces,
                    const SimParams& sim_params);
    virtual ~BaseTimeStepper();

    void addForce(int ind, double p, int limb_idx);
    void addForceShell(int ind, double p, int shell_limb_idx);

    virtual void initStepper();
    virtual void prepSystemForIteration();
    virtual void setZero();
    virtual void update();
    virtual void integrator() = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx) = 0;
    virtual void addJacobianShell(int ind1, int ind2, double p, int shell_limb_idx) = 0;
    virtual void addJacobianShell(int ind1, int ind2, double p, int shell_limb_idx1, int shell_limb_idx2) = 0;
    virtual void addJacobian(int ind1, int ind2, double p, int limb_idx1, int limb_idx2) = 0;
    virtual void updateSystemForNextTimeStep() = 0;
    virtual double stepForwardInTime() = 0;

    double* dx;
    double* force;
    Eigen::Map< Eigen::VectorXd> Force;
    Eigen::Map< Eigen::VectorXd> DX;
    Eigen::MatrixXd Jacobian;

    int freeDOF;
    int nDOF;
    std::vector<int> offsets;
    std::vector<int> offsets_shell;
    int iter = 0;


protected:
    int mappedInd, mappedInd1, mappedInd2;
    int offset;
    int offset_shell;
    double dt;
    double alpha = 1.0;

    std::vector<std::shared_ptr<ElasticRod>>& limbs;
    std::vector<std::shared_ptr<ElasticShell>>& shell_limbs;
    std::vector<std::shared_ptr<ElasticJoint>>& joints;
    std::vector<std::shared_ptr<BaseController>>& controllers;
    std::shared_ptr<ForceContainer> forces;
};

#endif
