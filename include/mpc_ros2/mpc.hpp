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

#ifndef MPC_HPP
#define MPC_HPP

#include <iostream>
#include <map>
#include <math.h>
#include <vector>
#include <tf2_eigen/tf2_eigen.hpp>
#include <cppad/ipopt/solve.hpp>

namespace MpcRos
{
typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;

class MPC
{
  public:
    MPC();
    MPC(const std::map<std::string, double> &params);
    void solve(Eigen::VectorXd state, std::vector<double>& outVec, Eigen::VectorXd coeffs);
    void operator()(ADvector& fg, const ADvector& vars, Eigen::VectorXd coeffs);

  private:
    double _dt, _refEtheta, _maxAngvel, _maxAccel, _boundValue;
    size_t _mpcStepsize, _refCte, _refVel, _wCte,
     _wEtheta, _wVel, _wAngvel, _wAccel, _wAngveld, _wAcceld;
    size_t _xStart, _yStart, _thetaStart, _vStart,
     _cteStart, _eThetaStart, _angvelStart, _accStart;
    CppAD::AD<double> cost_cte, cost_etheta, cost_vel;
    
};
} // namespace MpcRos
#endif