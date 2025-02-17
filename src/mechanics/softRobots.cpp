#include "softRobots.h"


softRobots::softRobots() = default;

softRobots::~softRobots() = default;


void softRobots::addLimb(const Vector3d& start, const Vector3d& end, int num_nodes,
                        double rho, double rod_radius, double youngs_modulus, double poisson_ratio, double mu) {
    limbs.push_back(make_shared<elasticRod>(num_limbs, start, end, num_nodes, rho,
                                            rod_radius, youngs_modulus, poisson_ratio, mu));
    num_limbs++;
    total_limbs++;
}


void softRobots::addLimb(const vector<Eigen::Vector3d> &nodes, double rho, double rod_radius, double youngs_modulus,
                        double poisson_ratio, double mu) {
    limbs.push_back(make_shared<elasticRod>(num_limbs, nodes, rho, rod_radius, youngs_modulus, poisson_ratio, mu));
    num_limbs++;
    total_limbs++;
}

void softRobots::addShellLimb(const vector<Vector3d>& nodes, const vector<vector<int> >& Face_Nodes, double rho, double thickness,
                             double youngs_modulus, double poisson_ratio, double mu) {
    shell_limbs.push_back(make_shared<elasticShell>(num_shell_limbs, nodes, Face_Nodes, rho, thickness, youngs_modulus, poisson_ratio, mu));
    num_shell_limbs++;
    total_limbs++;
}


void softRobots::createJoint(int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv-1;
    }
    joints.push_back(make_shared<elasticJoint>(node_idx, limb_idx, limbs));
}


void softRobots::addToJoint(int joint_idx, int limb_idx, int m_node_idx) {
    int node_idx = m_node_idx;
    if (m_node_idx == -1) {
        node_idx = limbs[limb_idx]->nv-1;
    }
    joints[joint_idx]->addToJoint(node_idx, limb_idx);
}


void softRobots::lockEdge(int limb_idx, int edge_idx) {
    shared_ptr<elasticRod> limb = limbs[limb_idx];
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx), edge_idx);
    limb->setVertexBoundaryCondition(limb->getVertex(edge_idx+1), edge_idx+1);
    limb->setThetaBoundaryCondition(0.0, edge_idx);
}

void softRobots::lockEdgeShell(int shell_limb_idx, int edge_idx) {
    shared_ptr<elasticShell> shell_limb = shell_limbs[shell_limb_idx];
    vector<int> edge_nodes = shell_limb->EdgeIsBet[edge_idx];
    shell_limb->setVertexBoundaryCondition(shell_limb->getVertex(edge_nodes[0]), edge_nodes[0]);
    shell_limb->setVertexBoundaryCondition(shell_limb->getVertex(edge_nodes[1]), edge_nodes[1]);
}

void softRobots::lockNodeShell(int shell_limb_idx, int node_idx) {
    shared_ptr<elasticShell> shell_limb = shell_limbs[shell_limb_idx];
    shell_limb->setVertexBoundaryCondition(shell_limb->getVertex(node_idx), node_idx);
}

void softRobots::applyInitialVelocities(int limb_idx, const vector<Vector3d> &velocities) {
    shared_ptr<elasticRod> limb = limbs[limb_idx];
    if (limb->nv != velocities.size()) {
        throw runtime_error("The number of nodes (" + to_string(limb->nv) +
                            ") and velocities (" + to_string(velocities.size()) + ") given did not match!");
    }
    for (int i = 0; i < limb->nv; i++) {
        limb->u.segment(4*i, 3) = limb->u0.segment(4*i, 3) = velocities.at(i);
    }
}


void softRobots::setup() {
    for (const auto& joint : joints)  {
        joint->setup();
    }
}


void softRobots::addController(const shared_ptr<baseController>& controller) {
    controllers.emplace_back(controller);
}
