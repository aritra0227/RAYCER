#ifndef SPHERE_H
#define SPHERE_H

#include "hittable.h"
#include "interval.h"
#include "vec3.h"

class sphere : public hittable
{
private:
    point3 center;
    double radius;
    shared_ptr<material> mat;

public:
    sphere(point3 _center, double _radius, shared_ptr<material> _material)
      : center(_center), radius(_radius), mat(_material) {}
    bool hit(const ray &r, interval ray_t, hit_record &rec) const override
    {
        vec3 oc = r.origin() - center;
        double a = r.direction().length_squared();
        double half_b = dot(oc, r.direction());
        double c = oc.length_squared() - radius * radius;

        double discriminant = half_b * half_b - a * c;
        if (discriminant < 0)
            return false;
        double sqrtd = sqrt(discriminant);

        // Find the nearest root that lies in the acceptable range.
        double root = (-half_b - sqrtd) / a;
        if (!ray_t.surrounds(root))
        {                                 //check if out of bounds
            root = (-half_b + sqrtd) / a; // hence check other root
            if (!ray_t.surrounds(root))   //check if still out of bounds
                return false;
        }

        rec.t = root;                                    //hits at time t
        rec.p = r.at(rec.t);                             //at point p
        rec.normal = (rec.p - center) / radius;          //calculate normal vector of surface
        rec.mat = mat;
        vec3 outward_normal = (rec.p - center) / radius; //make into unit vector
        rec.set_face_normal(r, outward_normal);

        return true;
    }
};
#endif