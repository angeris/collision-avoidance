/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : polynomial.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Apr 26, 2021.
  Description  : 3d polynomial trajectory with bezier support
**************************************************************************/

#ifndef __POLYNOMIAL_H__
#define __POLYNOMIAL_H__

#include <vector>
#include <Eigen/Dense>

class Polynomial {
 public:
    Polynomial(int order=0);
    Polynomial(std::vector<float> c, bool bezier=false);
    ~Polynomial();
    
    // set coefficents
    void setCoeff(std::vector<float>& c);
    void setCoeffBezier(std::vector<float>& c);

    // get values and derivitives
    float evaluate(float t);
    float derivitive(float t, int order=1);

 private:
    int _order;
    std::vector<float> _coeff;

};

class Polynomial3d {
 public:
    Polynomial3d(int order=0);
    Polynomial3d(std::vector<std::vector<float>> coeff);
    ~Polynomial3d();

    std::vector<float> evaluate(float t);
    std::vector<float> derivitive(float t, int order=1);
    
 private:
    Polynomial x;
    Polynomial y;
    Polynomial z;

};





#endif