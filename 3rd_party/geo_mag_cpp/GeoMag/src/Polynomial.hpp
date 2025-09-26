/**
 * @file  Polynomial.hpp
 * @author Kaiji Takeuchi
 * @brief
 * @version 0.1
 * @date 2023-12-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "Essential.hpp"

GEOMAG_NAMESPACE_BEGIN

struct Polynomial {
	static double deg1(double x, double a, double b) { return a + b * x; }

	static double deg2(double x, double a, double b, double c) { return a + (b + c * x) * x; }

	static double deg3(double x, double a, double b, double c, double d) { return a + (b + (c + d * x) * x) * x; }

	static double deg4(double x, double a, double b, double c, double d, double e) { return a + (b + (c + (d + e * x) * x) * x) * x; }

	static double deg5(double x, double a, double b, double c, double d, double e, double f) {
		return a + (b + (c + (d + (e + f * x) * x) * x) * x) * x;
	}

	static double deg6(double x, double a, double b, double c, double d, double e, double f, double g) {
		return a + (b + (c + (d + (e + (f + g * x) * x) * x) * x) * x) * x;
	}

	static double deg7(double x, double a, double b, double c, double d, double e, double f, double g, double h) {
		return a + (b + (c + (d + (e + (f + (g + h * x) * x) * x) * x) * x) * x) * x;
	}

	static double deg8(double x, double a, double b, double c, double d, double e, double f, double g, double h, double i) {
		return a + (b + (c + (d + (e + (f + (g + (h + i * x) * x) * x) * x) * x) * x) * x) * x;
	}

	static double deg9(double x, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j) {
		return a + (b + (c + (d + (e + (f + (g + (h + (i + j * x) * x) * x) * x) * x) * x) * x) * x) * x;
	}
};

GEOMAG_NAMESPACE_END