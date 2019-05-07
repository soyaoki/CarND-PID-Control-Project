#ifndef PID_H
#define PID_H

#include <vector>
#include <iostream>
#include <cmath>
#include <math.h>
#include <numeric>
#include <fstream>
#include <random>

double mod (double n, double M);

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(std::vector<double> p, bool opt_flg);
  void Init_th(std::vector<double> p, bool opt_flg);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate controll value by PID.
   * @output Controll value (steering angle)
   */
  double Cal_controll_val();
  
  double run(double best_err);
  double run_th(double best_err);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  bool _1st_loop_flg; // 1st loop flg

  /**
   * PID Coefficients
   */ 
  // Steer
  double Kp;
  double Ki;
  double Kd;
  // Throttle
  double Kp_th;
  double Ki_th;
  double Kd_th;
  
};

#endif  // PID_H