#include "Shape.h"
namespace pbrt {
	Shape::Shape(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation)
	{

	}
	Bounds3f Shape::WorldBound() const
	{
		return (*ObjectToWorld)(ObjectBound());
	}
}
