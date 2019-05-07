#include "PID.h"
using namespace std; 

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

// Steerting controller
void PID::Init(vector<double> p, bool opt_flg) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  // Initialize params
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  i_error = 0.0;
  _1st_loop_flg = true;
  
  std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;
  
  // Optimization by Twiddle
  if (opt_flg)
  {
    // Twiddling parameters
    vector<double> dp = {1.0,1.0,1.0};
    vector<double> p = {Kp,Ki,Kd};
    vector<double> p_opt = {Kp,Ki,Kd};
    int count = 1;
    double total_error = 0.0;
    double best_error = 9999999.0;
    double tol = 0.1;
    double sum_dp = accumulate(dp.begin(), dp.end(), 0.0);
    
//    while (sum_dp > tol || count < 1000 )
    while (sum_dp > tol )
    {
      std::cout << "---------------- Count: " << count << " ----------------" << std::endl;
      std::cout << "p: " << p[0] << ", " << p[1]<< ", " << p[2] << std::endl;
      std::cout << "dp: " << dp[0] << ", " << dp[1]<< ", " << dp[2] << std::endl;
      
      // PID controller for optimization
      PID pid_opt;
      pid_opt.Init(p,false);
      
      // 0: Kp, 1: Ki, 2: Kd
      for (int i = 0; i < p.size(); i++)
      {
        std::cout << "i: " << i << std::endl; // 0: Kp, 1: Ki, 2: Kd
        // p = p + dp
        p[i] += dp[i]; 
        // Controller by using new params 
        pid_opt.Init(p,false);
        // Simulation 
        total_error = pid_opt.run(best_error);
        
        if (total_error < best_error)
        {
          // If best error updates
          std::cout << "Best Error: " << best_error << std::endl;
          std::cout << "total_error < best_error !" << std::endl;
          // Update dp
          dp[i] *= 1.1;
          best_error = total_error;
          // Update output value p_opt
          p_opt[i] = p[i];
        }
        else
        {
          // p = p - dp
          p[i] -= 2*dp[i]; 
          // Controller by using new params 
          pid_opt.Init(p,false);
          // Simulation
          total_error = pid_opt.run(best_error);
          
          if (total_error < best_error)
          {
            // If best error updates
            std::cout << "Best Error: " << best_error << std::endl;
            std::cout << "total_error < best_error !" << std::endl;
            // Update dp
            dp[i] *= 1.1;
            best_error = total_error;
            // Update output value p_opt
            p_opt[i] = p[i];
          }
          else
          {
            // If best error doesnt update
            std::cout << "Best Error: " << best_error << std::endl;
            std::cout << "total_error > best_error..." << std::endl;
            p[i] += dp[i]; // p = p
            // Update dp
            dp[i] *= 0.9;            
            std::cout << "Kp: " << p[0] << ", Ki: " << p[1] << ", Kd: " << p[2] << std::endl;
          }
        }
      }
      sum_dp = accumulate(dp.begin(), dp.end(), 0.0);
      std::cout << "Sum of dp: " << sum_dp << std::endl;
      count++;
      std::cout << std::endl;
    }
    std::cout << "********* Final Params *********"<< std::endl;
    Kp = p_opt[0];
    Ki = p_opt[1];
    Kd = p_opt[2];
    std::cout << "Kp: " << Kp<< ", Ki: " << Ki << ", Kd: " << Kd << std::endl;    
  }
}

// Steerting controller simulation
double PID::run(double best_err){
  // Initial parameters
  double x = 0.0;
  double y = 1.0;
  double ori = 0.0;
  // double distance = 30.0 * 0.44704 * 0.02; // vel * dt [m]
  // int n = std::round(100/distance);
  double distance = 1.0;
  int n = 100;
  double cte = y;
  double p_err = cte;
  double i_err = 0.0;
  double d_err = 0.0;
  double total_error = 0.0;
  double length = 20.0;
  double tolerance = 0.001;
  double pi = 3.14159;
  double max_steering_angle = pi / 4.0;
  double pre_steer = 0.0;
  double steering_drift = 10.0 * pi/180;
  vector<double> x_vec;
  vector<double> y_vec;
  vector<double> ori_vec;
  std::random_device seed_gen;
  std::default_random_engine engine(seed_gen());
  
  for (int i = 0; i < 2*n; i++)
  {
    // sensing and calcurate errors
    cte = y;
    d_err = cte - p_err;
    i_err += cte;
    p_err = cte;
    
    // Calcurate steering angle
    double steer = - Kp * p_err - Ki * i_err - Kd * d_err;
    std::normal_distribution<> dist(steer, 5.0*pi/180);
    double steer_noise = dist(engine);
    // steer = steer_noise + steering_drift;
    double d_steer = steer - pre_steer;
    pre_steer = steer;
    // if (steer > max_steering_angle) { steer = max_steering_angle; }
    // if (steer < -max_steering_angle) { steer = -max_steering_angle; }
    
    // Execute motion
    double turn = sin(steer) * distance / length;
    
    // Calcurate trajectory
    if (abs(turn) < tolerance)
    {
      // If turn is very small value (almost goning straight), approximate by straight line motion
      x += distance * cos(ori);
      y += distance * sin(ori);
      ori = mod((ori + turn), (2.0 * pi));
//      std::cout << x << ", " << y << ", " << ori << std::endl;
    } else
    {
      // else, approximate bicycle model for motion
      double radius = distance / turn; // curveture
      double cx = x - (sin(ori) * radius);
      double cy = y + (cos(ori) * radius);
      ori = mod((ori + turn), (2.0 * pi));
      x = cx + (sin(ori) * radius);
      y = cy - (cos(ori) * radius);
//      std::cout << x << ", " << y << ", " << ori << ", " << radius << std::endl;
    }
    if (i > n)
    {    
      // cost function design : minimize cross truck error, steer operation value and operation change
      total_error += cte*cte + steer*steer + d_steer*d_steer;
    }
    
    // Add current x, y and orientation
    x_vec.push_back(x);
    y_vec.push_back(y);
    ori_vec.push_back(ori);
  }
  // If best error update
  if ( best_err > (total_error / n ) )
  {
    std::cout << "  x  |  y  |  orientation" << std::endl;
    // Loging
    std::ofstream log_file;
    log_file.open("optimized_params_res.csv");
    // Display
    for (int i =0; i < x_vec.size(); i++)
    {
      std::cout << x_vec[i] << ", " << y_vec[i] << ", " << ori_vec[i] << std::endl;
      log_file << x_vec[i] << ", " << y_vec[i] << ", " << ori_vec[i] << std::endl;
    }
  }
  return total_error / n;
}

// Throttle controller
void PID::Init_th(vector<double> p, bool opt_flg) {
  
  // Initialize params
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  i_error = 0.0;
  _1st_loop_flg = true;
  
  std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << std::endl;
  
  // Optimization by Twiddle
  if (opt_flg)
  {
    // Twiddling parameters
    vector<double> dp = {1.0,1.0,1.0};
    vector<double> p = {Kp,Ki,Kd};
    vector<double> p_opt = {Kp,Ki,Kd};
    int count = 1;
    double total_error = 0.0;
    double best_error = 9999999.0;
    double tol = 0.1;
    double sum_dp = accumulate(dp.begin(), dp.end(), 0.0);
    
//    while (sum_dp > tol || count < 1000 )
    while (sum_dp > tol )
    {
      std::cout << "---------------- Count: " << count << " ----------------" << std::endl;
      std::cout << "p: " << p[0] << ", " << p[1]<< ", " << p[2] << std::endl;
      std::cout << "dp: " << dp[0] << ", " << dp[1]<< ", " << dp[2] << std::endl;
      
      // PID controller for optimization
      PID pid_opt;
      pid_opt.Init_th(p,false);
      
      // 0: Kp, 1: Ki, 2: Kd
      for (int i = 0; i < p.size(); i++)
      {
        std::cout << "i: " << i << std::endl; // 0: Kp, 1: Ki, 2: Kd
        // p = p + dp
        p[i] += dp[i]; 
        // Controller by using new params 
        pid_opt.Init_th(p,false);
        // Simulation 
        total_error = pid_opt.run_th(best_error);
        
        if (total_error < best_error)
        {
          // If best error updates
          std::cout << "Best Error: " << best_error << std::endl;
          std::cout << "total_error < best_error !" << std::endl;
          // Update dp and best error
          dp[i] *= 1.1;
          best_error = total_error;
          // Update output value p_opt
          p_opt[i] = p[i];
        }
        else
        {
          // p = p - dp
          p[i] -= 2*dp[i]; 
          // Controller by using new params 
          pid_opt.Init_th(p,false);
          // Simulation
          total_error = pid_opt.run_th(best_error);
          
          if (total_error < best_error)
          {
            // If best error updates
            std::cout << "Best Error: " << best_error << std::endl;
            std::cout << "total_error < best_error !" << std::endl;
            // Update dp and best error
            dp[i] *= 1.1;
            best_error = total_error;
            // Update output value p_opt
            p_opt[i] = p[i];
          }
          else
          {
            // If best error doesnt update
            std::cout << "Best Error: " << best_error << std::endl;
            std::cout << "total_error > best_error..." << std::endl;
            p[i] += dp[i]; // p = p
            // Update dp
            dp[i] *= 0.9;            
            std::cout << "Kp: " << p[0] << ", Ki: " << p[1] << ", Kd: " << p[2] << std::endl;
          }
        }
      }
      sum_dp = accumulate(dp.begin(), dp.end(), 0.0);
      std::cout << "Sum of dp: " << sum_dp << std::endl;
      count++;
      std::cout << std::endl;
    }
    std::cout << "********* Final Params *********"<< std::endl;
    Kp = p_opt[0];
    Ki = p_opt[1];
    Kd = p_opt[2];
    std::cout << "Kp: " << Kp<< ", Ki: " << Ki << ", Kd: " << Kd << std::endl;    
  }
}

// Throttle controller simulation
double PID::run_th(double best_err){
  // Initial parameters
  double vel = 0.0;
  double target_vel = 30.0;
  int n = 100;
  double error = target_vel - vel;
  double p_err = error;
  double i_err = 0.0;
  double d_err = 0.0;
  double total_error = 0.0;
  double max_throttle = 1.0;
  double pre_throttle = 0.0;
  vector<double> throttle_vec;
  vector<double> vel_vec;
  
  for (int i = 0; i < n; i++)
  {
    // sensing and calcurate errors
    error = target_vel - vel;
    d_err = error - p_err;
    i_err += error;
    p_err = error;
    
    // Calcurate th & vel
    double throttle = - Kp * p_err - Ki * i_err - Kd * d_err;
    double d_throttle = throttle - pre_throttle;
    pre_throttle = throttle;
//    if (throttle > max_throttle) { throttle = max_throttle; }
//    if (throttle < -max_throttle) { throttle = -max_throttle; }
    vel += throttle;
    
    // Add current x, y and orientation
    throttle_vec.push_back(throttle);
    vel_vec.push_back(vel);
    
    // cost function : minimize velocity error, throttle operation value and throttle operation change
    total_error += error*error + throttle*throttle + d_throttle*d_throttle;
  }
  // If best error updates
  if ( best_err > (total_error / n ) )
  {
    std::cout << "  x  |  y  |  orientation" << std::endl;
    // Loging
    std::ofstream log_file;
    log_file.open("optimized_params_res_th.csv");
    // Display
    for (int i =0; i < throttle_vec.size(); i++)
    {
      std::cout << throttle_vec[i] << ", " << vel_vec[i] <<  std::endl;
      log_file << throttle_vec[i] << ", " << vel_vec[i] << std::endl;
    }
  }
  return total_error / n;
}

// Others
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  if (_1st_loop_flg)
  {
    p_error = cte;
    _1st_loop_flg = false;
  }
  
  d_error = cte - p_error; // d
  p_error = cte; // p
  i_error += cte; // i
  
  std::cout << "P_ERROR: " << p_error << ", I_ERROR: " << i_error << ", D_ERROR: " << d_error << std::endl;

}

double PID::Cal_controll_val() {
  /**
   * TODO: Calculate and return the total error
   */
  return -Kp * p_error - Ki * i_error - Kd * d_error;  // TODO: Add your total error calc here!
}

// modulo operator
double mod (double n, double M){
  return std::fmod((std::fmod(n,M) + M), M);
}