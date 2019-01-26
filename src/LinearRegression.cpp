#include "LinearRegression.h"

LinearRegression::LinearRegression(void){
	xSum=0;
	ySum=0;
	xxSum=0;
	xySum=0;
}


vector<float> LinearRegression::updateModel(float newX, float newY){
	xData.push_back(newX);
	yData.push_back(newY);

	// Calculate new set of sums
    xSum += newX;
    ySum += newY;
    xxSum += (newX * newX);
    xySum += (newX * newY);
    // cout << xSum << "\n";;

	// Caluculate slope and intercept
	slope = (yData.size() * xySum - xSum * ySum) / (yData.size() * xxSum - xSum * xSum);
    intercept = (ySum - slope * xSum) / yData.size();

    // Place the slope and intercept into a vector to return
    std::vector<float> res;
    res.push_back(slope);
    res.push_back(intercept);
    return res;
}


float LinearRegression::forwardCalculation(float x){
    return (slope * x) + intercept;
}


float LinearRegression::inverseCalculation(float y){
    return (y - intercept) / slope;
}