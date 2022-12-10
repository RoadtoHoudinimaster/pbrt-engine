#include "Shape.h"
namespace pbrt {
	Shape::Shape(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation)
		:ObjectToWorld(ObjectToWorld), WorldToObject(WorldToObject), reverseOrientation(reverseOrientation) ,transformSwapHandedness(ObjectToWorld->SwapHandedness()){
	}
	Bounds3f Shape::WorldBound() const
	{
		return (*ObjectToWorld)(ObjectBound());
	}
}
