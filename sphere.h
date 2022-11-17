#pragma once


#include"shape.h"

namespace pbrt
{
	class Sphere :public Shape
	{
	public:
		Sphere(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation, float radius, float zMin, float zMax, float phiMax)
			:Shape(ObjectToWorld, WorldToObject, reverseOrientation), radius(radius), zMin(Clamp(std::min(zMin, zMax) - radius, radius)), zMax(Clamp(std::max(zMin, zMax), -radius, radius)),
			thetaMin(std::acos(Clamp(zMin / radius, -1, 1))),
			thetaMax(std::acos(Clamp(zMax / radius, -1, 1))),
			phiMax(Radians(Clamp(phiMax, 0, 360))) {}
		Bounds3f ObjectBound()const;
		bool Intersect(const Ray& ray, float *tHit,SurfaceInteraction *isect,bool testAlphaTexture)const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture)const;
		float Area()const;
		Interaction Sample(const Point2f &u)const;
		//So the ref dec is white because the interaction part cannot be compiled
		Interaction Sample(const Interaction &ref, const Point2f &u)const;
		float Pdf(const Interaction &ref, const Vector3f &wi)const;

	private:
		const float radius;
		const float zMin, zMax;
		const float thetaMin, thetaMax, phiMax;
	};
}