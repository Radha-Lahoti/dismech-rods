#ifndef ELASTICBENDINGFORCESHELL_H
#define ELASTICBENDINGFORCESHELL_H

#include "mechanics/base_force.h"

class BaseTimeStepper;

class elasticBendingForceShell : public BaseForce
{
public:
    elasticBendingForceShell(const std::shared_ptr<SoftRobots>& m_soft_robots);
    ~elasticBendingForceShell() override;
    void computeForce(double dt) override;
    void computeForceAndJacobian(double dt) override;

private:
    void initValues(Eigen::Vector3d m_e0, Eigen::Vector3d m_e1 , Eigen::Vector3d m_e2, Eigen::Vector3d m_e3, Eigen::Vector3d m_e4);
    void computeGradTheta(Eigen::VectorXd& gradTheta);
    void computeHessTheta(Eigen::MatrixXd& hessTheta);
    double signedAngle(const Eigen::Vector3d &u, const Eigen::Vector3d &v, const Eigen::Vector3d &n);
    Eigen::Matrix3d s(Eigen::Matrix3d &mat);

    std::vector<double> EIMatrices;
    std::vector<int> hinge;
    double theta;
    Eigen::MatrixXd hessTheta;
    Eigen::VectorXd gradTheta;
    Eigen::MatrixXd Jbb;
    Eigen::VectorXd f;

    Eigen::Vector3d m_e0, m_e1, m_e2, m_e3, m_e4;

    Eigen::Vector3d m_nn1;
    Eigen::Vector3d m_nn2;
    Eigen::Vector3d m_m1;
    Eigen::Vector3d m_m2;
    Eigen::Vector3d m_m3;
    Eigen::Vector3d m_m4;
    Eigen::Vector3d m_m01;
    Eigen::Vector3d m_m02;


    double m_cosA1;
    double m_cosA2;
    double m_cosA3;
    double m_cosA4;

    double m_sinA1;
    double m_sinA2;
    double m_sinA3;
    double m_sinA4;

    double m_psi;
    double m_zeta;
    double m_xi;

    double m_h1;
    double m_h2;
    double m_h3;
    double m_h4;
    double m_h01;
    double m_h02;

    Eigen::Matrix3d Id3;
};

#endif