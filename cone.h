#pragma once 
#include"Shape.h"
namespace pbrt {
	class Cone :public Shape
	{
	public:
		Cone(const Transform* o2w, const Transform* w2o, bool reverseOrientation, float height, float radius, float phiMax);
		Bounds3f ObjectBound()const;
		bool Intersect(const& Ray ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture)const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture)const;
		float Area()const;
		Interaction Sample(const Point2f& u, float* pdf)const;
	protected:
		const float radius, height, phiMax;
	};
	st::shared_ptr<Cone> CreateConeShape(const Transform* o2w, const Transform* w2o, bool reverseOritation, const ParamSet& params);
}