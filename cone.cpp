#include "cone.h"

pbrt::Cone::Cone(const Transform* o2w, const Transform* w2o, bool reverseOrientation, float height, float radius, float phiMax)
    :Shape(o2w, w2o, reverseOrientation), radius(radius), height(height), phiMax(Radians(Clamp(phiMax, 0, 360))) {}

pbrt::Bounds3f pbrt::Cone::ObjectBound() const
{
    return Bounds3f(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
}

bool pbrt::Cone::Intersect(const Ray& r, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
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

bool pbrt::Cone::IntersectP(const Ray& r, bool testAlphaTexture) const
{
    float phi;
    Point3f pHit;
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    EFloat k = EFloat(radius) / EFloat(height);
    k = k * k;
    EFloat a = dx * dx + dy * dy - k * dz * dz;
    EFloat b = 2 * (dx * ox + dy * oy - k * dz * (oz - height));
    EFloat c = ox * ox + oy * oy - k * (oz - height) * (oz - height);


    //Solve quadratic equation for _t_ values
    EFloat t0, t1;
    if (!Quadratic(a.UpperBound(), b.UpperBound(), c.UpperBound(), &t0, &t1))return false;


    if (t0.UpperBound() > ray.tMax || t1.LowerBound() <= 0)return false;
    EFloat tShapeHit = t0;
    if (tShapeHit.LowerBound() <= 0) {
        tShapeHit = t1;
        if (tShapeHit.UpperBound() > ray.tMax)return false;
    }

    //Compute cone inverse mapping
    pHit = ray((float)tShapeHit);
    phi = std::atan2(pHit.y, pHit.x);
    if (phi < 0.)phi += 2 * Pi;
    if (pHit.z < 0 || pHit.z > height || phi > phiMax) {
        if (tShapeHit == t1)return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax)return false;
        pHit = ray((float)tShapeHit);
        phi = std::atan2(pHit.y, pHit.x);
        if (phi < 0.)phi += 2 * Pi;
        if (pHit.z<0 || pHit.z >height || phi > phiMax)return false;
    }
    return true;
}

float pbrt::Cone::Area() const
{
    return radius * std::sqrt((height * height) + (radius * radius)) * phiMax / 2;
}
pbrt::Interaction pbrt::Cone::Sample(const Point2f& u, float* pdf)const {
    return Interaction();
}
