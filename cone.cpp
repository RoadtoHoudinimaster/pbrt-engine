#include "cone.h"

pbrt::Cone::Cone(const Transform* o2w, const Transform* w2o, bool reverseOrientation, float height, float radius, float phiMax)
    :Shape(o2w, w2o, reverseOrientation), radius(radius), height(height), phiMax(Radians(Clamp(phiMax, 0, 360))) {}
{
}

Bounds3f pbrt::Cone::ObjectBound() const
{
    return Bounds3f(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
}

bool pbrt::Cone::Intersect(const& Ray ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
{
    float phi;
    Point3f pHit;
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat k = EFloat(radius) / EFloat(height);
    k = k * k;

}

bool pbrt::Cone::IntersectP(const Ray& ray, bool testAlphaTexture) const
{

}

float pbrt::Cone::Area() const
{

}

pbrt::Interaction pbrt::Cone::Sample(const Point2f& u, float* pdf) const
{

}
