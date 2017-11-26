#include "PID.h"
#include <cmath>
#include <iostream>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;
	p_error = d_error = i_error = 0.0;

	// Twiddle parameters	
	//dp = { 0.1*Kp,0.1*Kd,0.1*Ki };
	p = { Kp, Kd, Ki };
	dp = { Kp, Kd, Ki };
	isTwiddle = false;
	isAdd = false;
	isSub = false;
	step = 1;
	p_index = 2; 
	settling_steps = 100;
	eval_steps = 200;
	err = 0;
	best_error = std::numeric_limits<double>::max();
	thresh = 0.0001;
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
	if (step % (settling_steps + eval_steps) > settling_steps)
		err += pow(cte, 2);

	//To arrive at optimized parameters, use Twiddle
	//if (isTwiddle && step % (settling_steps + eval_steps) == 0)
	//{
	//	dp_sum = std::accumulate(dp.begin(), dp.end(), 0.0);
	//	int dp_size = dp.size()
	//		while (dp_sum > thresh)
	//		{
	//			for (size_t i = 0; i < dp; i++)
	//			{
	//				if (err < best_error)
	//				{
	//					best_error = err;
	//					if (step != settling_steps + eval_steps)
	//					{
	//						dp[i] *= 1.1;
	//					}
	//					isAdd = false;
	//					isSub - false;
	//				}

	//				if (!isAdd && !isSub)
	//				{
	//					p[i] += dp[i];
	//					isAdd = true;
	//				}
	//				else if (isAdd && !isSub)
	//				{
	//					p[i] -= 2 * dp[i];
	//					isSub = true;
	//				}
	//				else
	//				{
	//					p[i] += dp[i];
	//					dp[i] *= 0.9;
	//					isAdd = false;
	//					isSub = false;
	//				}
	//			}
	//			err = 0;
	//		}
	//}


	if (isTwiddle && step % (settling_steps + eval_steps) == 0) {
		cout << "Step: " << step << endl;
		cout << "Total Error: " << err << endl;
		cout << "Best eEror: " << best_error << endl;
		if (err < best_error) {
			best_error = err;
			//Skip on step 1
			if (step != settling_steps + eval_steps) {
				dp[p_index] *= 1.1;
			}
			//Forgoing a best practice loop due to unaviability of current cte at this point and incrementing
			//parameter index
			p_index = (p_index + 1) % 3;
			isAdd = false; 
			isSub = false;
		}
		if (!isAdd && !isSub) {			
			Add_dp(p_index, dp[p_index]);
			isAdd = true;
		}
		else if (isAdd && !isSub) {
			Add_dp(p_index, -2 * dp[p_index]);
			isSub = true;
		}
		else {
			Add_dp(p_index, dp[p_index]);
			dp[p_index] *= 0.9;
			//Increment
			p_index = (p_index + 1) % 3;
			isAdd = false;
			isSub = false;
		}
		err = 0;
	}
	step++;
}

double PID::TotalError() {
	return 0.0;  // ?
}

void PID::Add_dp(int index, double value) {
	if (index == 0) {
		Kp += value;
	}
	else if (index == 1) {
		Kd += value;
	}
	else if (index == 2) {
		Ki += value;
	}
}