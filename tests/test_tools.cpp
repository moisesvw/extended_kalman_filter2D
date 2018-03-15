#include <gtest/gtest.h>
#include "../src/Eigen/Dense"
#include "../src/tools.h"
#include <iostream>
#include <vector>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

TEST(Tools, RMSE) {
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

    Tools::Tools t;
	VectorXd expected(4);
	expected << 0.1,0.1,0.1,0.1;

    ASSERT_TRUE( t.CalculateRMSE(estimations, ground_truth).isApprox(expected) );
}

TEST(Tools, JACOBIAN) {
	VectorXd input(4);
	input << 1.0, 2.0, 0.2, 0.4;

    Tools::Tools t;
	MatrixXd expected(3,4);
	expected << 0.447214, 0.894427, 0, 0,
				-0.4, 0.2, 0, 0,
				0, 0, 0.447214, 0.894427;

    ASSERT_TRUE( t.CalculateJacobian(input).isApprox(expected, 0.00001) );
}
