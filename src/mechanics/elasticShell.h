#ifndef ELASTICSHELL_H
#define ELASTICSHELL_H

#include "global_definitions.h"

class ElasticShell
{
    // NOTE: probably could move more stuff to private
    public:
    ElasticShell(int limb_idx, const std::vector< Eigen::Vector3d>& nodes, const std::vector<std::vector<int> >& Face_Nodes, double rho, double thickness,
               double youngs_modulus, double poisson_ratio, double mu);
    ~ElasticShell();


    // utility functions
    void setVertexBoundaryCondition( Eigen::Vector3d position, int k);
    void prepareForIteration();
    double updateNewtonX(double *dx, int offset, double alpha=1.0);
    void updateGuess(double weight, double dt);

    int limb_idx;

    // utility functions
     Eigen::Vector3d getVertex(int k);
     Eigen::Vector3d getPreVertex(int k);
     Eigen::Vector3d getVelocity(int k);
     Eigen::Vector3d getTangent(int k);
    double getHingeAngle(int k);

    // Elastic stiffness values
    double poisson_ratio;
    double youngM; // Young's and shear modulus
    double EA;             // stretching stiffness
    double EI;             // bending stiffness

    int nv;                    // number of vertices
    int ne;                    // number of edges
    int nh;                    // number of hinges
    int nf;                    // number of faces
    int ndof;                  // number of degrees of freedom = 3*nv 
    int ncons;                 // number of constrained dof
    int uncons;                // number of unconstrained dof
    double rho;                // density
    double thickness;          // cross-sectional thickness of the shell
    std::vector<double> dm; // mass of each face
    double mu; // friction coefficient
    
    // Edge length
     Eigen::VectorXd edge_len;
    // reference lengths
     Eigen::VectorXd ref_len;
    // hinge angles
     Eigen::VectorXd phi;
    // intial reference hinge angles
     Eigen::VectorXd phi_bar;
    

    std::vector< Eigen::Vector3d> all_nodes;
    std::vector<std::vector<int> > Face_Nodes;
    std::vector< Eigen::Vector3d> Edges; 
    std::vector<std::vector<int> > Face_Edges;
    std::vector<std::vector<int> > sign_face_edges; 
    std::vector<std::vector<int> > EdgeIsBet; 
    std::vector<std::vector<int> > HingeIsBet;
    std::vector< Eigen::Vector3d> edge_avg_normals;


    // dof std::vector before time step
     Eigen::VectorXd x0;
    // dof std::vector after time step
     Eigen::VectorXd x;
    // dof std::vector for line search /***** what is this?
     Eigen::VectorXd x_ls;
    // velocity std::vector
     Eigen::VectorXd u;
     Eigen::VectorXd u0;

    // Face Normals
    std::vector< Eigen::Vector3d> normals;
    // Tangents
    Eigen::MatrixXd tangent;
    Eigen::MatrixXd tangent_old;

    // lumped mass
     Eigen::VectorXd mass_array;
    // lumped mass at unconstrained dofs
     Eigen::VectorXd mUncons;

    // boundary conditions
    int *isConstrained;
    int getIfConstrained(int k) const;
    int *unconstrainedMap;
    int *fullToUnconsMap;

    void updateMap();

    void freeVertexBoundaryCondition(int k);

    void addJoint(int node_num, bool remove_dof, int joint_node, int joint_limb);
    int unique_dof;
    int *isDOFJoint;
    int *isNodeJoint;
    int *isEdgeJoint;
    int *DOFoffsets;
    std::vector<std::pair<int, int>> joint_ids;

private:
    void setupMap(); //********* not understood this clearly, kept as is for shell for now.

    // NOTE: perhaps move these to util.h later?
    void setup();
    void computeElasticStiffness();
    void setMass();
    void setReferenceLength();
    void setPhiBar();
    void computeTangent();
    void computeEdgeLen();
    void computeHingeAngles();
    static double signedAngle(const  Eigen::Vector3d &u, const  Eigen::Vector3d &v, const  Eigen::Vector3d &n);
};

#endif
