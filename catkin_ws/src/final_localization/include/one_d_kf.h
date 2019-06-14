#pragma once

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>

extern bool verbose;

class OneDKalmanFilter{
 private:
  // Model:  x_{t} = a*x_{t-1} + b*u_{t} + epsilon_{t}
  // Sensor: z_{t} = c*x_{t} + delta_{t}
  double a_, b_, c_, k; // k: kalman gain
  double mu_, var_; // Now gaussian attribute
  bool ready; // Is filter ready?
  std::string filterName;
 public:
  OneDKalmanFilter(double a, double b, double c, double mu, double var, std::string name): 
    a_(a), b_(b), c_(c), mu_(mu), var_(var), filterName(name), ready(true)
  {
    if(verbose) {
      std::cout << std::fixed << std::showpoint << std::setprecision(3);
      std::cout << "\033[1;33m" << filterName << " ready \033[0m\n";
    }
  }
  OneDKalmanFilter(double a, double b, double c, std::string name):
    a_(a), b_(b), c_(c), filterName(name), ready(false){}
  void setInitialGaussian(double mu, double var) {
    mu_ = mu; var_ = var; ready = true;
    if(verbose) {
      std::cout << std::fixed << std::showpoint << std::setprecision(3);
      std::cout << "\033[1;33m" << filterName << " ready \033[0m\n";
    }
  }
  double getMean(void) {return mu_;}
  double getVariance(void) {return var_;}
  bool update(double u, double action_var, double z, double observation_var);
};

bool OneDKalmanFilter::update(double u, double action_var, double z, double observation_var){
  if(!ready) {std::cout << "\033[1;31mFilter not ready!\033[0m\n"; return false;}
  // Predict part
  double mu_bar = a_*mu_ + b_*u;
  double var_bar = a_*a_*var_ + action_var;
  // Update
  k = var_bar/(var_bar + observation_var);
  mu_ = mu_bar + k*(z-mu_bar);
  var_ = (1-k)*var_bar;
  if(verbose) {std::cout << "\033[1;33m"
                         << filterName 
                         << "|Raw: " << z
                         << "|Mean: " << mu_
                         << "|Variance: " << var_
                         << "|Gain: " << k 
                         << "\033[0m\n";}
  return true;
}
