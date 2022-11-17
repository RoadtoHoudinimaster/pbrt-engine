#pragma once 
#include<iostream>
#include<algorithm>
#include<cmath>
#include<limits>
#include<memory>
#include<string>
#include<vector>
//So to compare if the 2 float is equal, then we can use the numeric_limits::epsilon to compare the difference them!

namespace pbrt
{
	#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)
		static  float ShadowEpsilon = 0.0001f;
		static  float Pi = 3.14159265358979323846;
		static  float InvPi = 0.31830988618379067154;
		static  float Inv2Pi = 0.15915494309189533577;
		static  float Inv4Pi = 0.07957747154594766788;
		static  float PiOver2 = 1.57079632679489661923;
		static  float PiOver4 = 0.78539816339744830961;
		static  float Sqrt2 = 1.41421356237309504880;
	template<typename T>
	class Vector2;
	template<typename T>
	class Vector3;
	template<typename T>
	class Point3;
	template<typename T>
	class Point2;
	template<typename T>
	class Normal3;
	class Medium;
	class SurfaceInteration;
	class Transform;
	class AnimatedTransform;

	//Shape Class


	class EFloat;
	class Shape;
	class Sphere;
	struct Matrix4x4;
	
	//class Ray;
	inline float gamma(int n) {
		return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
	}
	inline double BitsToFloat(uint64_t ui)
	{
		double f;
		memcpy(&f, &ui, sizeof(uint64_t));
		return f;
	}
	inline uint64_t FloatToBits(double f)
	{
		uint64_t ui;
		memcpy(&ui, &f, sizeof(double));
		return ui;
	}
	inline float NextFloatUp(float v) {
		if (std::isinf(v) && v > 0.)return v;
		if (v == -0.f)return 0.f;
		uint64_t ui = FloatToBits(v);
		if (v >= 0)
			++ui;
		else
			--ui;
		return BitsToFloat(ui);
	}
	inline float NextFloatDown(float v) {
		if (std::isinf(v) && v < 0.)return v;
		if (v == 0.f)v = -0.f;
		uint64_t ui = FloatToBits(v);
		if (v > 0)
			--ui;
		else
			++ui;
		return BitsToFloat(ui);
	}
	template<typename T,typename U,typename V>
	inline T Clamp(T val, U low, V high) {
		if (val < low)
			return low;
		else if (val > high)
			return high;
		else return val;
	}
	inline float Lerp(float t, float v1, float v2)
	{
		return (1 - t) * v1 + t * v2;
	}
	inline float Radians(float deg) {
		return (Pi / 180) * deg;
	}
	inline float Degrees(float rad) {
		return (180 / Pi) * rad;
	}
	inline float Log2(float x) {
		const float invLog2 = 1.442695040888963387004650940071;
		return std::log(x) * invLog2;
	}
	inline bool Quadratic(float a, float b, float c, float* t0, float* t1) {
		double discrim = (double)b * (double)b - 4 * (double)a * (double)c;
		if (discrim < 0)return false;
		double rootDiscrim = std::sqrt(discrim);

		double q;
		if (b < 0)
			q = -.5 * (b - rootDiscrim);
		else 
			q = -.5 * (b + rootDiscrim);
		*t0 = q / a;
		*t1 = c / q;
		//according to the calculation,the t0 is the closest distance and the t1 is the further hot point!
		if (*t0 > * t1)std::swap(*t0, *t1);
		return true;
	}
}