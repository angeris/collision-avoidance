/* copyright[2020] <msl> <kunal shah>
**************************************************************************
  File Name    : Bezier.h
  Author       : Kunal Shah
                 Multi-Robot Systems Lab (MSL), Stanford University
  Contact      : k2shah@stanford.edu
  Create Time  : Apr 26, 2021.
  Description  : 3d Bezier trajectory with bezier support
**************************************************************************/

#ifndef __BEZIER_H__
#define __BEZIER_H__

#include <vector>
#include <map>
#include <Eigen/Dense>

class Bezier {
 public:
    Bezier(int order=0);
    ~Bezier();
    
    // set coefficents
    void setCoeff(std::vector<float>& c);

    // get values and derivitive
    float evaluate(float t);
    float derivitive(float t);

 private:
    int _order;
    std::vector<float> _coeff;

    // bernstein
    float bernstien(int i, int n, float t);
};

class Bezier3d {
 public:
    Bezier3d(int order=0);
    void setCoeff(std::map<char, std::vector<float>> coeff);
    ~Bezier3d();

    // componet polynomials 
    Bezier x;
    Bezier y;
    Bezier z;


    std::vector<float> evaluate(float t);
    std::vector<float> derivitive(float t);
    

};





#endif