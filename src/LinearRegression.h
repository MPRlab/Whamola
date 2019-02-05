/*
 * General Linear Regression class that can update its model on every new data point given. 
 * This is used for the Whamola instrument to map the relationship between tuning lever position and the frequency^2 produced by the string
 * 
 * Author: Sean O'Neil
 *
 */

#include <iostream>
#include <vector>

using namespace std;

class LinearRegression{
public:

	LinearRegression(void);

	vector<float> xData;
	vector<float> yData;

	vector<float> updateModel(float newX, float newY);

	float forwardCalculation(float x);
	float inverseCalculation(float y);

private:

	float xSum, ySum, xxSum, xySum, slope, intercept;


};