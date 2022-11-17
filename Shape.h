#pragma once
#include"pbrt.h"
#include"Transform.h"
#include"interaction.h"
namespace pbrt {
	class Shape {
	public:
		Shape(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation);
		virtual ~Shape();
		virtual Bounds3f ObjectBound() const = 0;
		virtual Bounds3f WorldBound()const;
		virtual bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture = true)const = 0;
		virtual bool IntersectP(const Ray& ray, bool testAlphaTexture = true)const {
			float tHit = ray.tMax;
			SurfaceInteraction isect;
			return Intersect(ray, &tHit, &isect, testAlphaTexture);
		}
		virtual float Area()const = 0;
		virtual Interaction Sample(const Point2f& u)const = 0;
		virtual Pdf(const Interaction&)const {
			return 1 / Area();
		}
		virtual Interaction Sample(const Interaction &ref, const Point2f& u)const = 0;
		virtual float Pdf(const Interaction& ref, const Vector3f& wi)const;

		//Data
		const Transform* ObjectToWorld, * WorldToObject;
		const bool reverseOrientation;
		const bool transformSwapHandedness;
	};
	
	class EFloat {
	public:
		EFloat() {}
		EFloat(float v, float err = .0f) :v(v), err(err) {
			ld = v;
		}
		EFloat operator+(EFloat f)const {
			EFloat r;
			r.v = v + f.v;
			r.err = err + f.err + gamma(1) * (std::abs(v + f.v) + err + f.err);
			return r;
		}
		explicit operator float()const { return v; }
		float GetAbsoluteError()const { return err; }
		float UpperBound()const { return NextFloatUp(v + err); }
		float LowerBound()const { return NextFloatDown(v - err); }
		float GetRelativeError()const {
			return std::abs(ld - v) / ld;
		}
		long double PreciseValue()const {
			return ld;
		}
		
	private:
		float v;
		float err;
	};
	
}
