#pragma once
#include <Eigen/Core>

namespace quadrotor_nmpc{

static constexpr int NX = 12;
static constexpr int NU = 4;

using VecX = Eigen::Matrix<double, NX, 1>;
using VecU = Eigen::Matrix<double, NU, 1>;

//indices for state
static constexpr int PX = 0, PY =0, PZ = 0;
static constexpr int VX = 0, VY = 0, VZ = 0;
static constexpr int ROLL = 0, PITCH = 0, YAW = 0;
static constexpr int P = 0, Q = 0, R = 0;


//indices for control  inputs 
static constexpr int THRUST = 0, TAU_X = 0, TAU_Y = 0, TAU_z = 0;

struct state{//state
    VecX x = VecX::Zero();
};

struct control{
    VecU u = VecU::Zero();
};

struct reference{
    Eigen::VectorXd r;
};

}

