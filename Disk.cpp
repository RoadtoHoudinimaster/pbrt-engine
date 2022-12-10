#include "Disk.h"

pbrt::Bounds3f pbrt::Disk::ObjectBound() const
{
    return Bounds3f(Point3f(-radius,-radius,height),Point3f(radius,radius,height));
}

bool pbrt::Disk::Intersect(const Ray& r, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
{
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);
    if (ray.d.z == 0)
        return false;
    float tShapeHit = (height - ray.o.z) / ray.d.z;
    if (tShapeHit <= 0 || tShapeHit >= ray.tMax)
        return false;
    //See if hit point is inside disk radius and thetaMax
    Point3f pHit = ray(tShapeHit);
    float dist2 = pHit.x * pHit.x + pHit.y * pHit.y;
    if (dist2 > radius * radius || dist2 < innerRadius * innerRadius)
        return false;
    float phi = std::atan2(pHit.y, pHit.x);
    if (phi < 0)phi += 2 * Pi;
    if (phi > phiMax)return false;
    //calculate the parametric representation
    float u = phi / phiMax;
    float rHit = std::sqrt(dist2);
    float oneMinusV = ((rHit - innerRadius) / (radius - innerRadius));
    float v = 1 - oneMinusV;
    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv = Vector3f(pHit.x, pHit.y, 0.) * (innerRadius - radius) / rHit;
    Normal3f dndu(0, 0, 0), dndv(0, 0, 0);
    //Refine disk intersection point
    pHit.z = height;
    //I mean I am really hate the design of the Vector3f
    //Compute error Bounds for disk intersection
    Vector3f pError(0, 0, 0);
    //Initialize SurfaceInteraction from parameteric information,this is tricky again
    *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d, dpdu, dpdv, dndu, dndv, ray.time, this));
    //Update tHit for quadric intersection
    //Now need to figure out the parametric of the shape!
    *tHit = (float)tShapeHit;
    return true;
}

bool pbrt::Disk::IntersectP(const Ray& ray, bool testAlphaTexture) const
{
    return false;
}

float pbrt::Disk::Area() const
{
    return phiMax * 0.5 * (radius * radius - innerRadius * innerRadius);
}

pbrt::Interaction pbrt::Disk::Sample(const Point2f& u) const
{
    return Interaction();
}
