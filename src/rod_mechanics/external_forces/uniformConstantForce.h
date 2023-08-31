#ifndef UNIFORM_CONSTANT_FORCE_H
#define UNIFORM_CONSTANT_FORCE_H

#include "rod_mechanics/baseForce.h"

class baseTimeStepper;

class uniformConstantForce : public baseForce
{
public:
    uniformConstantForce(const vector<shared_ptr<elasticRod>>& m_limbs,
                         const vector<shared_ptr<elasticJoint>>& m_joints);
    ~uniformConstantForce() override;

    void add_force_to_limb(int limb_idx, const Vector3d& force);

    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    vector<pair<int, Vector3d>> limb_force_pairs;
};

#endif
