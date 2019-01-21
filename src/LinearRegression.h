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