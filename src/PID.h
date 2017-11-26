#ifndef PID_H
#define PID_H
#include <vector>


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Twiddle Parameters
  */
  std::vector<double> p;
  std::vector<double> dp;
  bool isTwiddle, isAdd, isSub;
  int step, p_index;
  int settling_steps, eval_steps;
  double err, best_error, thresh, dp_sum;


  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Add dp to PID parameters
  */
  void Add_dp(int index, double value);
};

#endif /* PID_H */
