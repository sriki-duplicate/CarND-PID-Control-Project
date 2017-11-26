#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0;
	i_error = 0;
	d_error = 0;
	isTwiddle = true;
	isAdd = false;
	isSub = false;
	//p = { Kp, Ki, Kd };
	p = { 0, 0, 0 };
	dp = { 1, 1, 1 };
	step = 1;
	settling_steps = 100;
	eval_steps = 500;
	err = 0;
	best_error = std::numeric_limits<double>::max();
	thresh = 0.001;
}

void PID::UpdateError(double cte) {
	//Setting p_error to cte on step 1 here so the d_error doesn't start off big
	if (step == 1)
		p_error = cte;

	//Setting the errors
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	
	//Based on the Parameter Optimization lecture video, we are using a settling period (100 steps) before we 
	//update the total CTE
	if (step % (settling_steps+eval_steps) > settling_steps)
		err += pow(cte, 2);


	//To arrive at optimized parameters, use Twiddle
	if (isTwiddle && step % (settling_steps + eval_steps) == 0)
	{
		dp_sum = std::accumulate(dp.begin(), dp.end(), 0.0);
		int dp_size = dp.size()
		while (dp_sum > thresh)
		{
			for (size_t i = 0; i < dp; i++)
			{
				if (err < best_error)
				{
					best_error = err;
					if (step != settling_steps + eval_steps)
					{
						dp[i] *= 1.1;
					}
					isAdd = false;
					isSub - false;
				}
				
				if (!isAdd && !isSub)
				{
					p[i] += dp[i];
					isAdd = true;
				}
				else if (isAdd && !isSub)
				{
					p[i] -= 2*dp[i];
					isSub = true;
				}
				else
				{
					p[i] += dp[i];
					dp[i] *= 0.9;
					isAdd = false;
					isSub = false;
				}
			}
			err = 0;
			cout << "new parameters" << endl;
			cout << "P: " << p[0] << ", I: " << p[1] << ", D: " << p[2] << endl;
		}
	}
	
}

double PID::TotalError() {
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

