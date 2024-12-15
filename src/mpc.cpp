/*
 * MIT License
 * 
 * Copyright (c) 2024 Mustafa Ege Kural
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
*/

#include "mpc_ros2/mpc.hpp"


namespace MpcRos
{

/**
* @brief MPC class default constructor
*/
MPC::MPC() 
{
  // Set default values
  _dt          = 0.1;  // Timestep in sec
  _mpcStepsize = 20;   // MPC predict horizon
  _refCte      = 0;    // Cross-tracking error
  _refVel      = 0;    // Ref velocity m/s
  _refEtheta   = 0.5;  // Ref heading error 
  _wCte        = 100;  // Cross-tracking error weight
  _wEtheta     = 100;  // Heading error weight 
  _wVel        = 1;    // Velocity weight
  _wAngvel     = 100;  // Angular vel (rad/s)
  _wAccel      = 50;   // Acceleration weight 
  _wAngveld    = 0;    // Change in angular vel
  _wAcceld     = 0;    // Change in angular vel weight
  _maxAngvel   = 3.0;  // Maximal angvel radian (~30 deg)
  _maxAccel    = 1.0;  // Maximal accelaration/throttle
  _boundValue  = 1.0e3;// Bound value for other variables

  // Adjust indexes for the solution vector
  _xStart      = 0;
  _yStart      = _xStart      + _mpcStepsize;
  _thetaStart  = _yStart      + _mpcStepsize;
  _vStart      = _thetaStart  + _mpcStepsize;
  _cteStart    = _vStart      + _mpcStepsize;
  _eThetaStart = _cteStart    + _mpcStepsize;
  _angvelStart = _eThetaStart + _mpcStepsize;
  _accStart    = _angvelStart + _mpcStepsize - 1;
}

/**
* @brief MPC class constructor 
*        Intialize with parameter map
*/
MPC::MPC(const std::map<std::string, double> &params)
{
  // Load from the parameter map
  _dt          = params.at("DT"); 
  _mpcStepsize = params.at("STEPS");
  _refCte      = params.at("REF_CTE");
  _refVel      = params.at("REF_ETHETA");
  _refEtheta   = params.at("REF_V");
  _wCte        = params.at("W_CTE");
  _wEtheta     = params.at("W_EPSI");
  _wVel        = params.at("W_V");
  _wAngvel     = params.at("W_ANGVEL");
  _wAccel      = params.at("W_ACC");
  _wAngveld    = params.at("W_ANGVELD");
  _wAcceld     = params.at("W_ACCD");
  _maxAngvel   = params.at("ANGVEL");
  _maxAccel    = params.at("MAXACC");
  _boundValue  = params.at("BOUNDV");
  
  // Adjust indexes for the solution vector
  _xStart      = 0;
  _yStart      = _xStart      + _mpcStepsize;
  _thetaStart = _yStart      + _mpcStepsize;
  _vStart      = _thetaStart + _mpcStepsize;
  _cteStart    = _vStart      + _mpcStepsize;
  _eThetaStart = _cteStart    + _mpcStepsize;
  _angvelStart = _eThetaStart + _mpcStepsize;
  _accStart    = _angvelStart + _mpcStepsize - 1;
}


/**
* @brief Operator overload for cost function
*        Evaluates the objective function and constraints using the syntax
*/
void MPC::operator()(ADvector& objFunc, const ADvector& vars, Eigen::VectorXd coeffs)
{
  // objFunc[0] for cost function
  objFunc[0] = 0;
  cost_cte =  0;
  cost_etheta = 0;
  cost_vel = 0;

  /*
  Single Objective Optimization:
      - Solvers like IPOPT require a single scalar objective to optimize.
      - Combining individual costs into one scalar allows the solver to assess the "goodness" of a solution as a single value.
  Weighted Contributions:
      - Different aspects of the vehicle's behavior (e.g., trajectory following, control effort, smooth transitions) are represented as separate terms.
      - Each term is weighted (e.g., _wCte, _wEtheta, etc.) to reflect its relative importance in the overall optimization.

  * If following the trajectory is critical, _wCte (weight for cross-track error) is high.
  * If smooth control is important, weights like _wAngveld (rate of change of steering) are increased.
  
  Trade-offs:
      - Combining costs with weights allows balancing between competing objectives. For example:
      - Reducing cross-track error may require higher control effort.
  Balancing these ensures the optimal solution aligns with priorities (e.g., accuracy vs. efficiency).
  */

  for (int i = 0; i < _mpcStepsize; i++) 
  {
    objFunc[0] += _wCte    * CppAD::pow(vars[_cteStart + i]    - _refCte, 2);
    objFunc[0] += _wEtheta * CppAD::pow(vars[_eThetaStart + i] - _refEtheta, 2);
    objFunc[0] += _wVel    * CppAD::pow(vars[_vStart + i]      - _refVel, 2);

    cost_cte    +=  _wCte    * CppAD::pow(vars[_cteStart + i]    - _refCte, 2);
    cost_etheta +=  _wEtheta * CppAD::pow(vars[_eThetaStart + i] - _refEtheta, 2); 
    cost_vel    +=  _wVel    * CppAD::pow(vars[_vStart + i]      - _refVel, 2); 
  }
  std::cout << "-----------------------------------------------" << std::endl;
  std::cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta  << ", " << cost_vel << std::endl;
  

  // Minimize the use of actuators.
  for (int i = 0; i < _mpcStepsize - 1; i++) {
    objFunc[0] += _wAngvel * CppAD::pow(vars[_angvelStart + i], 2);
    objFunc[0] += _wAccel * CppAD::pow(vars[_accStart + i], 2);
  }
  std::cout << "cost of actuators: " << objFunc[0] << std::endl; 

  // Minimize the value gap between sequential actuations.
  for (int i = 0; i < _mpcStepsize - 2; i++) {
    objFunc[0] += _wAngveld * CppAD::pow(vars[_angvelStart + i + 1] - vars[_angvelStart + i], 2);
    objFunc[0] += _wAcceld * CppAD::pow(vars[_accStart + i + 1] - vars[_accStart + i], 2);
  }
  std::cout << "cost of gap: " << objFunc[0] << std::endl; 
  

  // objFunc[x] for constraints
  // Initial constraints
  objFunc[1 + _xStart] = vars[_xStart];
  objFunc[1 + _yStart] = vars[_yStart];
  objFunc[1 + _thetaStart] = vars[_thetaStart];
  objFunc[1 + _vStart] = vars[_vStart];
  objFunc[1 + _cteStart] = vars[_cteStart];
  objFunc[1 + _eThetaStart] = vars[_eThetaStart];

  // Add system dynamic model constraint
  for (int i = 0; i < _mpcStepsize - 1; i++)
  {
      // The state at time t+1 .
      CppAD::AD<double> x1 = vars[_xStart + i + 1];
      CppAD::AD<double> y1 = vars[_yStart + i + 1];
      CppAD::AD<double> theta1 = vars[_thetaStart + i + 1];
      CppAD::AD<double> v1 = vars[_vStart + i + 1];
      CppAD::AD<double> cte1 = vars[_cteStart + i + 1];
      CppAD::AD<double> etheta1 = vars[_eThetaStart + i + 1];

      // The state at time t.
      CppAD::AD<double> x0 = vars[_xStart + i];
      CppAD::AD<double> y0 = vars[_yStart + i];
      CppAD::AD<double> theta0 = vars[_thetaStart + i];
      CppAD::AD<double> v0 = vars[_vStart + i];
      CppAD::AD<double> cte0 = vars[_cteStart + i];
      CppAD::AD<double> etheta0 = vars[_eThetaStart + i];

      // Only consider the actuation at time t.
      //CppAD::AD<double> angvel0 = vars[_angvelStart + i];
      CppAD::AD<double> w0 = vars[_angvelStart + i];
      CppAD::AD<double> a0 = vars[_accStart + i];


      //CppAD::AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      CppAD::AD<double> f0 = 0.0;
      for (int i = 0; i < coeffs.size(); i++) 
      {
        f0 += coeffs[i] * CppAD::pow(x0, i);
      }

      //CppAD::AD<double> trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
      CppAD::AD<double> trj_grad0 = 0.0;
      for (int i = 1; i < coeffs.size(); i++) 
      {
        trj_grad0 += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0)
      }
      trj_grad0 = CppAD::atan(trj_grad0);


      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `CppAD::AD<double>` and use of `CppAD`!
      // This is also CppAD can compute derivatives and pass
      // these to the solver.
      // TODO: Setup the rest of the model constraints
      objFunc[2 + _xStart + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
      objFunc[2 + _yStart + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
      objFunc[2 + _thetaStart + i] = theta1 - (theta0 +  w0 * _dt);
      objFunc[2 + _vStart + i] = v1 - (v0 + a0 * _dt);
      objFunc[2 + _cteStart + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
      objFunc[2 + _eThetaStart + i] = etheta1 - ((theta0 - trj_grad0) + w0 * _dt);
  }
  
}


/**
* @brief MPC solver
*/
void MPC::solve(Eigen::VectorXd state, std::vector<double>& outVec, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  const double x = state[0];
  const double y = state[1];
  const double theta = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double etheta = state[5];

  /* Set the number of model variables and constraints based on state variables
  *  Includes both model state vars and input(actuator) vars
  *  num = mpc_timesteps x num_of_states + (mpc_timesteps - 1) x num_of_input 
  *  For example: If the state is a 4 element vector, the actuators is a 2
  *  element vector and there are 10 timesteps. The number of variables is:
  *  4 * 10 + 2 * 9
  */
  size_t n_constraints = _mpcStepsize * state.size();
  size_t n_vars = n_constraints + (_mpcStepsize - 1) * outVec.size();
  
  

 double a;
  outVec[0] = a;
  outVec[1] = a;
}

} // namespace MpcRos