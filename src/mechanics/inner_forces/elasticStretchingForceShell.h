#ifndef ELASTICSTRETCHINGFORCESHELL_H
#define ELASTICSTRETCHINGFORCESHELL_H

#include "mechanics/base_force.h"

class BaseTimeStepper;

class elasticStretchingForceShell : public BaseForce
{
public:
    elasticStretchingForceShell(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~elasticStretchingForceShell() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    double len, refLength;
    double epsX;
    Eigen::Vector3d u;
    Eigen::Vector3d dxx;
    Eigen::Vector3d f;
    Eigen::Matrix3d Id3;
    Eigen::Matrix3d M0;
    Eigen::Matrix<double,1,3> v;
    Eigen::Matrix<double,6,6> Jss;
    
    double EA;
    int ind, ind1, ind2;	
};

#endif
