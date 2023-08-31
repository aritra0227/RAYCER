#ifndef MATERIAL_H
#define MATERIAL_H

#include "utilities.h"
#include "colour.h"
#include "hittable.h"

class hit_record;

class material
{
public:
    virtual ~material() = default;

    virtual colour emit_light() const
    {
        return colour(0, 0, 0);
    }

    virtual bool scatter(
        const ray &r_in, const hit_record &rec, colour &attenuation, ray &scattered) const = 0;
};

class lambertian : public material
{
private:
    colour albedo;

public:
    lambertian(const colour &a) : albedo(a) {}

    bool scatter(const ray &r_in, const hit_record &rec, colour &attenuation, ray &scattered)
        const override
    {
        (void)r_in;
        vec3 scatter_direction = rec.normal + random_unit_vector();
        // Catch degenerate scatter direction
        if (scatter_direction.near_zero())
            scatter_direction = rec.normal;
        scattered = ray(rec.p, scatter_direction);
        attenuation = albedo;
        return true;
    }
};

class metal : public material
{
private:
    colour albedo;
    double fuzz;

public:
    metal(const colour &a, double f) : albedo(a), fuzz(f < 1 ? f : 1) {}

    bool scatter(const ray &r_in, const hit_record &rec, colour &attenuation, ray &scattered)
        const override
    {
        vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        scattered = ray(rec.p, reflected + fuzz * random_unit_vector());
        attenuation = albedo;
        return true;
    }
};

class dielectric : public material
{
private:
    double ir; // Index of Refraction

    static double reflectance(double cosine, double ref_idx)
    {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - ref_idx) / (1 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1 - r0) * pow((1 - cosine), 5);
    }

public:
    dielectric(double index_of_refraction) : ir(index_of_refraction) {}

    bool scatter(const ray &r_in, const hit_record &rec, colour &attenuation, ray &scattered)
        const override
    {
        attenuation = colour(1.0, 1.0, 1.0);
        double refraction_ratio = rec.front_face ? (1.0 / ir) : ir;
        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

        bool cannot_refract = refraction_ratio * sin_theta > 1.0;
        vec3 direction;

        if (cannot_refract || reflectance(cos_theta, refraction_ratio) > random_double())
            direction = reflect(unit_direction, rec.normal);
        else
            direction = refract(unit_direction, rec.normal, refraction_ratio);

        scattered = ray(rec.p, direction);
        return true;
    }
};

class light : public material
{
public:
    light(colour c) : light_colour(c) {}

    bool scatter(const ray &r_in, const hit_record &rec, colour &attenuation, ray &scattered)
        const override
    {
        (void)r_in;
        (void)rec;
        (void)attenuation;
        (void)scattered;
        return false;
    }

    colour emit_light() const override
    {
        return light_colour;
    }

private:
    colour light_colour;
};

#endif