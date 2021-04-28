/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : Bezier.cpp
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Apr 26, 2021.
  Description  : 3d Bezier trajectory 
**************************************************************************/

#include<carp_ros/bezier.h>
#include <boost/math/special_functions/binomial.hpp>
#include<iostream>


Bezier::Bezier(int order){
    _order = order;
    _coeff = std::vector<float>(_order+1, 0.0f);
}

Bezier::~Bezier() {
}

void Bezier::setCoeff(std::vector<float>& c){
    for (int i = 0; i< c.size(); i++) {
        _coeff[i] = c[i];
    }
}

float Bezier::bernstien(int i, int n, float t){
    //evalute the ith bernstein polynomila of _order at t
    float bcoeff = boost::math::binomial_coefficient<float>(n, i);
    return bcoeff * pow(t, i) * pow(1.-t, n-i);
}

float Bezier::evaluate(float t){
    float val = 0;
    for (int i = 0; i < _coeff.size(); i++) {
        val += bernstien(i, _order, t) * _coeff[i];
    }
    return val;
}

float Bezier::derivitive(float t){
    // evalute the 1st deriviative at t
    float val = 0;
    for (int i = 0; i < _coeff.size()-1; i++) {
        val += bernstien(i, _order-1, t) * (_coeff[i+1]-_coeff[i]);
    }
    return _order*val;
}


Bezier3d::Bezier3d(int order){
    Bezier x(order);
    Bezier y(order);
    Bezier z(order);
}
Bezier3d::~Bezier3d() {
}

void Bezier3d::setCoeff(std::map<char, std::vector<float>> coeff) {
    x.setCoeff(coeff['x']);
    y.setCoeff(coeff['y']);
    z.setCoeff(coeff['z']);
}

