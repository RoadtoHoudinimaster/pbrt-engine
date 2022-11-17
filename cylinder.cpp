#include "cylinder.h"

Bounds3f pbrt::Cylinder::ObjectBound() const
{
    return Bounds3f(Point3f(-radius,-radius,zMin),Point3f(radius,radius,zMax));
}

bool pbrt::Cylinder::Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
{
    float phi;
    Point3f pHit;
    Vector3f oErr, dErr;
    Ray ray = (*WorldToObject)(r, &oErr, &dErr);

    //Compute the quadratic cylinder coefficients
    EFloat ox(ray.o.x, oErr.x), oy(ray.o.y, oErr.y), oz(ray.o.z, oErr.z);
    EFloat dx(ray.d.x, dErr.x), dy(ray.d.y, dErr.y), dz(ray.d.z, dErr.z);
    //compute the coefficients
    EFloat a = dx * dx + dy * dy;
    EFloat b = 2 * (dx * ox + dy * oy);
    EFloat c = ox * ox + oy * oy - EFloat(radius) * EFloat(radius);
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
    float hitRad = std::sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
    pHit.x *= radius / hitRad;
    pHit.y *= radius / hitRad;
    if (phi < 0)phi += 2 * Pi;
    phi = std::atan2(pHit.y, pHit.x);
    //Same: test cylinder intersection against clipping parameters.
    //For the incompete cylinder collision!
    if (pHit.z<zMin || pHit.z >zMax || phi > phiMax) {
        if (tShapeHit == t1)return false;
        tShapeHit = t1;
        if (t1.UpperBound() > ray.tMax)return false;
        pHit = ray((float)tShapeHit);
        float hitRad = std::sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
        pHit.x *= radius / hitRad;
        pHit.y *= radius / hitRad;
        phi = std::atan2(pHit.y, pHit.x);
        if (phi < 0)phi += 2 * Pi;
        if (pHit.z<zMin || pHit.z>zMax || phi > phiMax)return false;
    }
    float u = phi / phiMax;
    float v = (pHit.z - zMin) / (zMin - zMax);
    //same deriviative calculation method as the sphere
    Vector3f dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
    Vector3f dpdv(0, 0, zMax - zMin);

    //Compute cylinder normal deriative
    Vector3f d2Pduu = -phiMax * phiMax * Vector3f(pHit.x, pHit.y, 0);
    Vector3f d2Pduv(0, 0, 0), d2Pdvv(0, 0, 0);
    //COmpute the coefficient for the Normal deriatives
    float E = Dot(dpdu, dpdu);
    float F = Dot(dpdu, dpdv);
    float G = Dot(dpdv, dpdv);
    Vector3f N = Normalize(Cross(dpdu, dpdv));
    float e = Dot(N, d2Pduu);
    float f = Dot(N, d2Pduv);
    float g = Dot(N, d2Pdvv);
    float invEGF2 = 1 / (E * G - F * F);
    Normal3f dndu = Normal3f((f * F - e * G) * invEGF2 * dpdu + (e * F - f * E) * invEGF2 * dpdu);
    Normal3f dndv = Normal3f((g * F - f * G) * invEGF2 * dpdu + (f * F - g * E) * invEGF2 * dpdv);
    Vector3f pError = gamma(3) * Abs(Vector3f(pHit.x, pHit.y, 0));
    //Initialize SurfaceInteraction from parameter information

    *isect = (*ObjectToWorld)(SurfaceInteraction(pHit, pError, Point2f(u, v), -ray.d, dpdu, dpdv, dndu, dndv, ray.time, this));
    *tHit = (float)tShapeHit;
    return true;
}

float pbrt::Cylinder::Area() const
{
    return(zMax-zMin)* radius * phiMax;
}
