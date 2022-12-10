#include"sphere.h"
//This chapater is really interesting!
namespace pbrt
{
	Bounds3f Sphere::ObjectBound() const
	{
		return Bounds3f(Point3f(-radius,-radius,zMin),
					Point3f(radius,radius,zMax));
	}

	bool Sphere::Intersect(const Ray& r, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
	{
		float phi;
		Point3f pHit;
		Vector3f oErr, dErr;
		Ray ray = (*WorldToObject)(r, &oErr, &dErr);
		EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
		EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
		EFloat a = dx * dx + dy * dy + dz * dz;
		EFloat b = 2.0f * (dx * ox + dy * oy + dz * oz);
		EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);

		EFloat t0, t1;
		//Is this the compiler error?
		if (!Quadratic(a.UpperBound(), b.UpperBound(), c.UpperBound(), &t0, &t1))
			return false;
		if (t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0)
			return false;
		EFloat tShapeHit = t0;
		if (tShapeHit.LowerBound() <= 0) {
			tShapeHit = t1;
			if (tShapeHit.UpperBound() > ray.tMax)
				return false;
		}
		//Compute sphere hit position and theta
		pHit = ray((float)tShapeHit);
		//Refine sphere intersection point
		pHit *= radius / Distance(pHit, Point3f(0, 0, 0));

		if (pHit.x == 0 && pHit.y == 0)pHit.x = 1e-5f * radius;
		phi = std::atan2(pHit.y, pHit.x);
		if (phi < 0)phi += 2 * Pi;
		//test sphere intersection clipping parameters,for the sphere that is not complete!
		if ((zMin > -radius && pHit.z < zMin) ||
			(zMax < radius) && pHit.z > zMax || phi > phiMax) {
			//this test is interesting!
			//when the top situation happens,the hit event will not count,need to be recalculated!
			//when the tShapeHit is t1,means there is no more hit point available!
			if (tShapeHit == t1)return false;

			if (t1.UpperBound() > ray.tMax)return false;
			//means the t0 will not work,switch back to t1!
			tShapeHit = t1;
			pHit = ray((float)tShapeHit);
			if (pHit.x == 0 && pHit.y == 0)pHit.x = 1e-5f * radius;
			phi = std::atan2(pHit.y,pHit.x);
			if (phi < 0)
				phi += 2 * Pi;
			//how to master the detail problem ,this is a hard trick,this test thinking is impressive!
			if ((zMin > -radius && pHit.z < zMin) || 
				(zMax <radius && pHit.z >zMax) || phi > phiMax)
				return false;
		}
		//Find the parametric representation of sphere hit
		float u = phi / phiMax;
		float theta = std::acos(Clamp(pHit.z / radius, -1, 1));
		float v = (theta - thetaMin) / (thetaMax - thetaMin);
		//Compute the differentiation of the (u,v)->P
		//Here replace angle with uv by setting theta = u * thetaMax;
		//Same for the delta :delta = deltaMin + v * (deltaMax-deltaMin);
		float zRadius = std::sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
		float invZRadius = 1 / zRadius;
		float cosPhi = pHit.x * invZRadius;
		float sinPhi = pHit.y * invZRadius;
		Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
		Vector3f dpdv = (thetaMax - thetaMin) * Vector3f(pHit.z * cosPhi, pHit.z * sinPhi, -radius * std::sin(theta));
		//Compute the differentiation of the (u,v)->N
		Vector3f d2Pduu = -phiMax * phiMax * Vector3f(pHit.x, pHit.y, 0);
		Vector3f d2Pduv = (thetaMax - thetaMin) * pHit.z * phiMax * Vector3f(-sinPhi, cosPhi, 0.);
		Vector3f d2Pdvv = -(thetaMax - thetaMin) * (thetaMax - thetaMin) * Vector3f(pHit.x, pHit.y, pHit.z);

		float E = Dot(dpdu, dpdu);
		float F = Dot(dpdu, dpdv);
		float G = Dot(dpdv, dpdv);
		Vector3f N = Normalize(Cross(dpdu, dpdv));
		float e = Dot(N, d2Pduu);
		float f = Dot(N, d2Pduv);
		float g = Dot(N, d2Pdvv);
		//Compute the differentiation of the (u,v)->N from the coefficients
		float invEGF2 = 1 / (E * G - F * F);
		Normal3f dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu + (e * F - f * E) * invEGF2 * dpdv);
		Normal3f dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu + (f * F - g * E) * invEGF2 * dpdv);
		//Compute error Bounds for sphere intersection
		Vector3f pError = gamma(5) * Abs((Vector3f)pHit);
		//Initialize SurfaceInteraction from parameter information
		*isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d, dpdu, dpdv, dndu, dndv, ray.time, this));
		//Update tHit for quadric intersection
		*tHit = (float)tShapeHit;
		return true;
		
	}

	bool Sphere::IntersectP(const Ray& r, bool testAlphaTexture) const
	{
		float phi;
		Point3f pHit;
		Vector3f oErr, dErr;
		Ray ray = (*WorldToObject)(r, &oErr, &dErr);
		EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
		EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);

		EFloat a = dx * dx + dy * dy + dz * dz;
		EFloat b = 2 * (dx * ox + dy * oy + dz * oz);
		EFloat c = ox * ox + oy * oy + oz * oz - EFloat(radius) * EFloat(radius);
		EFloat t0, t1;
		if (!Quadratic(a, b, c, &t0, &t1))
			return false;
		if (t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0)
			return false;
		EFloat tShapeHit = t0;
		if (tShapeHit.LowerBound() <= 0) {
			tShapeHit = t1;
			if (tShapeHit.UpperBound() > ray.tMax)
				return false;
			

		}
		pHit = ray((float)tShapeHit);
		pHit *= radius / Distance(pHit, Point3f(0, 0, 0));
		if (pHit.x == 0 && pHit.y == 0)pHit.x = 1e-5f * radius;
		phi = std::atan2(pHit.y, pHit.x);
		if (phi < 0)phi += 2 * Pi;
		if ((zMin > -radius && pHit.z < zMin) ||
			(zMax <  radius && pHit.z > zMax) || phi > phiMax) {
			if (tShapeHit == t1) return false;
			if (t1.UpperBound() > ray.tMax) return false;
			tShapeHit = t1;
			if ((zMin > -radius && pHit.z < zMin) ||
				(zMax <  radius && pHit.z > zMax) || phi > phiMax)
				return false;
		}
		return true;
	}
	float Sphere::Area() const
	{
		return phiMax * radius * (zMax - zMin);
	}
}