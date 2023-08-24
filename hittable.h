#ifndef HITTABLE_H
#define HITTABLE_H

#include "ray.h"

class hit_record
{
public:
    point3 p;
    vec3 normal;
    double t;
    bool front_face;

    /**
     * This function sets the surface normal always pointing outward from the surface
     * or pointing against the incident ray
     * @param: outward_normal: Must be of unit length
    */
    void set_face_normal(const ray &r, const vec3 &outward_normal)
    {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

class hittable
{
public:
    virtual ~hittable() = default;

    virtual bool hit(const ray &r, double ray_tmin, double ray_tmax, hit_record &rec) const = 0;
};

#endif