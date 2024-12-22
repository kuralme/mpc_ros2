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
#include <tuple>
#include <Eigen/Core>
#include <Eigen/QR>
#include <cppad/ipopt/solve.hpp>

namespace MpcRos
{
class MPC
{
  public:
    MPC();
    MPC(const std::map<std::string, double> &params);
    std::tuple<std::vector<std::tuple<double, double>>, std::vector<double>>
     solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
    
  private:
    double _maxAngvel, _maxAccel, _boundValue;
    size_t _mpcStepsize, _xStart, _yStart, _thetaStart, _vStart,
     _cteStart, _eThetaStart, _angvelStart, _accelStart;
    std::map<std::string, double> _mpcParams_;
};
} // namespace MpcRos
#endif