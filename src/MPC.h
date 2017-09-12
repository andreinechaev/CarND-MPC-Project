#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// TODO: Set the timestep length and duration
const double Dt = 0.1;
const size_t N = 20;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Set desired cte, epsi and speed
// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 45.5 mph.
const double REF_CTE = 0;
const double REF_EPSI = 0;
const double REF_V = 45.5;

// Set weights parameters for the cost function
const double W_CTE = 198.4;
const double W_EPSI = 192.32;
const double W_V = 0.261;
const double W_DELTA = 6;
const double W_A = 7.;
const double W_DDELTA = 2000.;
const double W_DA = 0.1;

// Set lower and upper limits for variables.
const double DED25RAD = 0.436332; // 25 deg in rad, used as delta bound
const double MAXTHR = 1.0; // Maximal a value
const double BOUND = 1.0e19; // Bound value for other variables

class MPC {
public:
    MPC();

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

    vector<double> mpc_x;
    vector<double> mpc_y;
};

#endif /* MPC_H */
