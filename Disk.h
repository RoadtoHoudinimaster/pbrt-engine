#pragma once
#include"Shape.h"
namespace pbrt {
	//The disk is the cutting face of cylinder
	class Disk:public Shape
	{
	public:
		Disk(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation, float height, float radius, float innerRadius, float phiMax)
			:Shape(ObjectToWorld, WorldToObject, reverseOrientation), height(height), radius(radius), innerRadius(innerRadius), phiMax(Radians(Clamp(phiMax, 0, 360))) {}
		Bounds3f ObjectBound()const;
		bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture)const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture)const;
		float Area()const;
		Interaction Sample(const Point2f& u)const;
	private:
		const float height, radius, innerRadius, phiMax;

	};
}