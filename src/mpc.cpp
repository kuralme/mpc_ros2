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
* @brief Single Objective Function Optimization Class
*        The FG_eval class is used to evaluate the objective function and constraints 
*        for the optimization problem. The class is used by the CppAD library to
*        compute the gradients of the objective function and constraints.
*        The class is a functor that overloads the operator() method to evaluate the
*        objective function and constraints.
*        The class is templated on the AD type, which is the type used by CppAD to
*        represent the independent and dependent variables.
*        https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm
* 
*/
class FG_eval 
{
  public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;
    double _dt, _refCte, _refVel, _refEtheta, _maxAngvel, _maxAccel, _boundValue, 
      _wCte, _wEtheta, _wVel, _wAngvel, _wAccel, _wAngveld, _wAcceld;
    int _mpcStepsize, _xStart, _yStart, _thetaStart, _vStart,
      _cteStart, _eThetaStart, _angvelStart, _accelStart;
    CppAD::AD<double> cost_cte, cost_etheta, cost_vel;
    typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

    /**
    * @brief FG Eval class constructor
    *        
    */
    FG_eval(Eigen::VectorXd coeffs) 
    {
      this->coeffs = coeffs;

      // Set default values
      _dt          = 0.1;  // Timestep in sec
      _mpcStepsize = 20;   // MPC predict horizon
      _refCte      = 0.0;  // Cross-tracking error
      _refVel      = 0.0;  // Ref velocity m/s
      _refEtheta   = 0.5;  // Ref heading error 
      _wCte        = 100;  // Cross-tracking error weight
      _wEtheta     = 100;  // Heading error weight 
      _wVel        = 1.0;  // Velocity weight
      _wAngvel     = 100.0;// Angular vel (rad/s)
      _wAccel      = 50.0; // Acceleration weight 
      _wAngveld    = 0.0;  // Change in angular vel
      _wAcceld     = 0.0;  // Change in angular vel weight
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
      _accelStart  = _angvelStart + _mpcStepsize - 1;
    }

    /**
    * @brief Parameter loader using the map
    */
    void LoadParameters(const std::map<std::string, double> &params)
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
      _thetaStart  = _yStart      + _mpcStepsize;
      _vStart      = _thetaStart  + _mpcStepsize;
      _cteStart    = _vStart      + _mpcStepsize;
      _eThetaStart = _cteStart    + _mpcStepsize;
      _angvelStart = _eThetaStart + _mpcStepsize;
      _accelStart  = _angvelStart + _mpcStepsize - 1;
    }
    
    /**
    * @brief Evaluate the objective function and constraints using the syntax
    *
    *  Solvers like IPOPT require a single scalar objective to optimize.
    *  Combining individual costs into one scalar allows the solver to assess the "goodness" 
    *  of a solution as a single value.
    *
    *  Weighted Contributions:
    *   - Different aspects of the vehicle's behavior (e.g., trajectory following,control
    *     effort, smooth transitions) are represented as separate terms
    *   - Each term is weighted (e.g., _wCte, _wEtheta, etc.) to reflect its relative
    *     importance in the overall optimization.
    *
    *  If following the trajectory is critical, _wCte (weight for cross-track error) is high.
    *  If smooth control is important, weights like _wAngveld (rate of change of steering) 
    *  are increased. Balancing trade-offs ensures the optimal solution aligns with  
    *  priorities (e.g., accuracy vs. efficiency).
    *
    *  Trade-offs:
    *   - Combining costs with weights allows balancing between competing objectives.
    *   - Reducing cross-track error may require higher control effort.
    *
    */
    void operator()(ADvector& fg, const ADvector& vars)
    {
      // fg[0] for cost function
      fg[0] = 0;
      cost_cte =  0;
      cost_etheta = 0;
      cost_vel = 0;

      for (int i = 0; i < _mpcStepsize; i++)
      {
        fg[0] += _wCte    * CppAD::pow(vars[_cteStart + i]    - _refCte   , 2);
        fg[0] += _wEtheta * CppAD::pow(vars[_eThetaStart + i] - _refEtheta, 2);
        fg[0] += _wVel    * CppAD::pow(vars[_vStart + i]      - _refVel   , 2);

        cost_cte    +=  _wCte    * CppAD::pow(vars[_cteStart + i]    - _refCte   , 2);
        cost_etheta +=  _wEtheta * CppAD::pow(vars[_eThetaStart + i] - _refEtheta, 2);
        cost_vel    +=  _wVel    * CppAD::pow(vars[_vStart + i]      - _refVel   , 2);
      }
      std::cout << "-----------------------------------------------" << std::endl;
      std::cout << "cost_cte, etheta, velocity: " << cost_cte << ", " << cost_etheta  << ", " << cost_vel << std::endl;
      
      // Minimize the actuator usage
      for (int i = 0; i < _mpcStepsize - 1; i++) {
        fg[0] += _wAngvel * CppAD::pow(vars[_angvelStart + i], 2);
        fg[0] += _wAccel  * CppAD::pow(vars[_accelStart + i]   , 2);
      }
      std::cout << "cost of actuators: " << fg[0] << std::endl;

      // Minimize the value gap between sequential actuations
      for (int i = 0; i < _mpcStepsize - 2; i++) {
        fg[0] += _wAngveld * CppAD::pow(vars[_angvelStart + i + 1] - vars[_angvelStart + i], 2);
        fg[0] += _wAcceld  * CppAD::pow(vars[_accelStart + i + 1]    - vars[_accelStart + i]   , 2);
      }
      std::cout << "cost of gap: " << fg[0] << std::endl; 
      
      // fg[x] for constraints
      // Initial constraints
      fg[1 + _xStart] = vars[_xStart];
      fg[1 + _yStart] = vars[_yStart];
      fg[1 + _thetaStart] = vars[_thetaStart];
      fg[1 + _vStart] = vars[_vStart];
      fg[1 + _cteStart] = vars[_cteStart];
      fg[1 + _eThetaStart] = vars[_eThetaStart];

      // Add system dynamic model constraint
      for (int i = 0; i < _mpcStepsize - 1; i++)
      {
        // The state at time t
        CppAD::AD<double> x0      = vars[_xStart + i];
        CppAD::AD<double> y0      = vars[_yStart + i];
        CppAD::AD<double> theta0  = vars[_thetaStart + i];
        CppAD::AD<double> v0      = vars[_vStart + i];
        CppAD::AD<double> cte0    = vars[_cteStart + i];
        CppAD::AD<double> etheta0 = vars[_eThetaStart + i];

        // The state at time t+1
        CppAD::AD<double> x1      = vars[_xStart + i + 1];
        CppAD::AD<double> y1      = vars[_yStart + i + 1];
        CppAD::AD<double> theta1  = vars[_thetaStart + i + 1];
        CppAD::AD<double> v1      = vars[_vStart + i + 1];
        CppAD::AD<double> cte1    = vars[_cteStart + i + 1];
        CppAD::AD<double> etheta1 = vars[_eThetaStart + i + 1];

        // Only consider the actuation at time t
        CppAD::AD<double> w0 = vars[_angvelStart + i];
        CppAD::AD<double> a0 = vars[_accelStart + i];

        // f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
        CppAD::AD<double> f0 = 0.0;
        for (int i = 0; i < coeffs.size(); i++) 
        {
          f0 += coeffs[i] * CppAD::pow(x0, i);
        }

        // trj_grad0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));
        CppAD::AD<double> trj_grad0 = 0.0;
        for (int i = 1; i < coeffs.size(); i++) 
        {
          trj_grad0 += i*coeffs[i] * CppAD::pow(x0, i-1); // f'(x0)
        }
        trj_grad0 = CppAD::atan(trj_grad0);

        // Minimize the model constraints
        fg[2 + _xStart + i]      = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
        fg[2 + _yStart + i]      = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
        fg[2 + _thetaStart + i]  = theta1 - (theta0 +  w0 * _dt);
        fg[2 + _vStart + i]      = v1 - (v0 + a0 * _dt);
        fg[2 + _cteStart + i]    = cte1 - ((f0 - y0) + (v0 * CppAD::sin(etheta0) * _dt));
        fg[2 + _eThetaStart + i] = etheta1 - ((theta0 - trj_grad0) + w0 * _dt);
      }
    }
};
/**
* @brief MPC class default constructor
*/
MPC::MPC() 
{
  // Set default values
  _mpcStepsize = 20;   // MPC predict horizon
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
  _accelStart  = _angvelStart + _mpcStepsize - 1;
}

/**
* @brief MPC class constructor 
*        Intialize with parameter map
*/
MPC::MPC(const std::map<std::string, double> &params)
: _mpcParams_(params)
{
  _mpcStepsize = _mpcParams_.at("STEPS");
  _maxAngvel   = _mpcParams_.at("ANGVEL");
  _maxAccel    = _mpcParams_.at("MAXACC");
  _boundValue  = _mpcParams_.at("BOUNDV");
  
  // Adjust indexes for the solution vector
  _xStart      = 0;
  _yStart      = _xStart      + _mpcStepsize;
  _thetaStart  = _yStart      + _mpcStepsize;
  _vStart      = _thetaStart  + _mpcStepsize;
  _cteStart    = _vStart      + _mpcStepsize;
  _eThetaStart = _cteStart    + _mpcStepsize;
  _angvelStart = _eThetaStart + _mpcStepsize;
  _accelStart  = _angvelStart + _mpcStepsize - 1;
}

/**
* @brief MPC solver
*        Solve the MPC problem and return the trajectory and control commands
*        Uses the IPOPT library to solve the optimization problem
*/
std::tuple<std::vector<std::tuple<double, double>>, std::vector<double>>
MPC::solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  const double x      = state[0];
  const double y      = state[1];
  const double theta  = state[2];
  const double v      = state[3];
  const double cte    = state[4];
  const double eTheta = state[5];

  /* Set the number of model variables and constraints based on state variables
  *  Includes both model state vars and input(actuator) vars
  *  num = mpc_timesteps x num_of_states + (mpc_timesteps - 1) x num_of_input 
  *  For example: If the state is a 4 element vector, the actuators is a 2
  *  element vector and there are 10 timesteps. The number of variables is:
  *  4 * 10 + 2 * 9
  */
  size_t nConstraints = _mpcStepsize * state.size();
  size_t nVars = nConstraints + (_mpcStepsize - 1) * 2;
  
  // Crate vectors for 
  //  - Optimization variables initial values
  //  - Lower and upper boundaries, all non-actuators upper and lowerlimits
  //  - Constraints bounds vectors to ensure that the system starts from the current state
  typedef CPPAD_TESTVECTOR(double) Dvector;
  Dvector vars(nVars);
  Dvector vars_lowerbound(nVars);
  Dvector vars_upperbound(nVars);
  Dvector constraints_lowerbound(nConstraints);
  Dvector constraints_upperbound(nConstraints);


  // ============================================================
  // Independent variables - SHOULD BE 0 besides initial state
  for (size_t i = 0; i < nVars; i++) 
  {
    vars[i] = 0;
  }
  // Set the initial values
  vars[_xStart]      = x;
  vars[_yStart]      = y;
  vars[_thetaStart]  = theta;
  vars[_vStart]      = v;
  vars[_cteStart]    = cte;
  vars[_eThetaStart] = eTheta;

  // ============================================================
  // Variable bounds - Set all to the max negative and positive
  for (size_t i = 0; i < _angvelStart; i++) 
  {
    vars_lowerbound[i] = -_boundValue;
    vars_upperbound[i] = _boundValue;
  }
  for (size_t i = _angvelStart; i < _accelStart; i++) 
  {
    vars_lowerbound[i] = -_maxAngvel;
    vars_upperbound[i] = _maxAngvel;
  }
  for (size_t i = _accelStart; i < nVars; i++)  
  {
    vars_lowerbound[i] = -_maxAccel;
    vars_upperbound[i] = _maxAccel;
  }

  // ============================================================
  // Constraints bounds - Should be 0 besides initial state.
  for (size_t i = 0; i < nConstraints; i++)
  {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[_xStart]      = x;
  constraints_lowerbound[_yStart]      = y;
  constraints_lowerbound[_thetaStart]  = theta;
  constraints_lowerbound[_vStart]      = v;
  constraints_lowerbound[_cteStart]    = cte;
  constraints_lowerbound[_eThetaStart] = eTheta;
  constraints_upperbound[_xStart]      = x;
  constraints_upperbound[_yStart]      = y;
  constraints_upperbound[_thetaStart]  = theta;
  constraints_upperbound[_vStart]      = v;
  constraints_upperbound[_cteStart]    = cte;
  constraints_upperbound[_eThetaStart] = eTheta;
  // ============================================================


  /* Print options for IPOPT solver
  * NOTE: Setting sparse to true allows the solver to take advantage
  *       of sparse routines, this makes the computation MUCH FASTER. If you
  *       can uncomment 1 of these and see if it makes a difference or not but
  *       if you uncomment both the computation time should go up in orders of
  *       magnitude.
  */
  std::string printOpt;
  printOpt += "Integer print_level  0\n"; // Uncomment for more print information
  printOpt += "Sparse  true        forward\n";
  printOpt += "Sparse  true        reverse\n";
  printOpt += "Numeric max_cpu_time   0.5\n";

  // Create object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  fg_eval.LoadParameters(_mpcParams_);

  // Solve MPC problem
  CppAD::ipopt::solve_result<Dvector> solution;
  CppAD::ipopt::solve<Dvector, FG_eval>(
    printOpt, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);

  // Check the results
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  if (!ok) {
    std::cerr << "MPC solve failed!" << std::endl;
    return {{}, {}}; // Return empty results in case of failure
  }

  auto cost = solution.obj_value;
  std::cout << "-----------------------------------------------" << std::endl;
  std::cout << "------------ Total Cost(solution): " << cost << "------------" << std::endl;
  std::cout << "-----------------------------------------------" << std::endl;

  std::vector<std::tuple<double, double>> mpcTraj;
  for (size_t i = 0; i < _mpcStepsize; i++) {
    double x = solution.x[_xStart + i];
    double y = solution.x[_yStart + i];
    mpcTraj.emplace_back(x, y);
  }

  std::vector<double> controlCommands;
  controlCommands.push_back(solution.x[_angvelStart]);
  controlCommands.push_back(solution.x[_accelStart]);

  return {mpcTraj, controlCommands};
}

} // namespace MpcRos