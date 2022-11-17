#pragma once 
#include"shape.h"

namespace pbrt
{ 
	class Cylinder:public Shape
	{
	public:
		Cylinder(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation, float radius, float zMin, float zMax, float phiMax)
			:Shape(ObjectToWorld, WorldToObject, reverseOrientation), radius(radius), zMin(std::min(zMin, zMax)), zMax(std::max(zMin, zMax)), phiMax(Radians(Clamp(phiMax, 0, 360))) {}
		Bounds3f ObjectBound()const;
		bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture)const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture)const;
		float Area()const;
		Interaction Sample(const Point2f& u)const;
	private:
		const float radius, zMin, zMax, phiMax;
	};
}