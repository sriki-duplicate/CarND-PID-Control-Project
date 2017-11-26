#ifndef PID_H
#define PID_H

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
  int step, param_index;
  int settling_steps, eval_steps;
  double err, best_error, thresh, dp_sum;
  bool isTwiddle, isAdd, isSub;


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
  * Convenience function for adding amount (dp) to a PID controller parameter based on index
  */
  void AddToParameterAtIndex(int index, double amount);
};

#endif /* PID_H */
