#pragma once

#include"pbrt.h"
namespace pbrt {
	template<typename T>
	class Texture {
		virtual T Evaluate(const SurfaceInteraction&)const = 0;
		virtual ~Texture() {}
	};
}