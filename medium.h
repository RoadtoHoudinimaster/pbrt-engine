#pragma once 
#include<iostream>
using namespace std;

namespace pbrt
{
	//Class Used for describing the atmosphere of the forest!
	class Medium {
	public:
		virtual ~Medium() {}
	};

	//Medium Interface
	struct MediumInterface {
		MediumInterface() :inside(nullptr), outside(nullptr) {}
		MediumInterface(const Medium* medium) :inside(medium), outside(medium) {}
		MediumInterface(const Medium* inside, const Medium* outside)
			:inside(inside), outside(outside) {}
		bool IsMediumTransition()const { return inside != outside; }
		const Medium* inside, * outside;
	};
}