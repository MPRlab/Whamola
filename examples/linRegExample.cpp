#include <iostream>
#include <vector>
#include "LinearRegression.h"

LinearRegression lr;


int main(){


	std::vector<float> xData;
	std::vector<float> yData;

	xData.push_back(2.12);
	yData.push_back(3.95);

	std::vector<float> result = lr.updateModel(2.12, 3.95);
	std::cout << "slope: " << result[0] << "\tintercept: " << result[1] << "\n";

	xData.push_back(-4);
	yData.push_back(1.15);
	
	std::vector<float> result2 = lr.updateModel(-4, 1.15);
	std::cout << "slope: " << result2[0] << "\tintercept: " << result2[1] << "\n";

	xData.push_back(6.5);
	yData.push_back(6.25);

	std::vector<float> result3 = lr.updateModel(6.5, 6.25);
	std::cout << "slope: " << result3[0] << "\tintercept: " << result3[1] << "\n";
}