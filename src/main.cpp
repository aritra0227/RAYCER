#include "bvh.h"
#include "camera.h"
#include "colour.h"
#include "interval.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "obj_parser.h"
#include "sphere.h"
#include "triangle.h"
#include "utilities.h"
#include <chrono>
#include "mesh.h"
using namespace std::chrono;
using namespace std;


/**
 * Responsible for constructing world of hittable objects
 * Then call render
*/
int main()
{
    // hittable_list world;
    BVH world;

    // auto material_ground = make_shared<dielectric>(colour(1, 1, 1));
    auto material_ground = make_shared<dielectric>(1.52);
    auto material_center = make_shared<metal>(colour(0.96, 0.60, 0.5), 0.0);
    auto material_right = make_shared<metal>(colour(1, 0.75, 0.80), 0.0);
    auto material_front_right = make_shared<metal>(colour(1, 0.75, 0.80), 0.0);
    auto material_back = make_shared<metal>(colour(0.996, 0.8516, 0.73), 0.0);
    auto material_back_back = make_shared<metal>(colour(0.996, 0.8516, 0.73), 0.25);
    auto material_small_front = make_shared<metal>(colour(0.9375, 0.5, 0.5), 0.10);
    auto material_big_right = make_shared<metal>(colour(0.996, 0.41, 0.70), 0);
    auto material_small_front_right = make_shared<metal>(colour(0.859375, 0.078125, 0.234375), 0.05);
    // auto material_light = make_shared<light>(colour(1.0,1.0,1.0));
    // auto mat_metal = make_shared<metal>(colour(0.7, 0.6, 0.5), 0.0);
    auto mat_metal = make_shared<metal>(colour((float)180/256, (float)189/256, (float)199/256), 0.0);
    // // world.add(make_shared<mesh>(npolys, faceIndex, vertsIndex, P, material_center));

    // auto ball_color = make_shared<lambertian>(colour(0.996, 0.41, 0.70));
    // auto lambertian_center = make_shared<lambertian>(colour(0.96, 0.60, 0.5));
    // // auto material_left = make_shared<dielectric>(1.5);
    // // auto material_right = make_shared<metal>(colour(0.8, 0.6, 0.2), 0.0);
    // world.add(make_shared<sphere>(point3(0, 120, -2.0), 101.5, material_light));

    world.add(make_shared<sphere>(point3(0.0, -102, -1.0), 100.0, mat_metal));
    world.add(make_shared<sphere>(point3(0.0, 0.0, -1.0), 0.5, material_center));
    world.add(make_shared<sphere>(point3(1.50, 0.5, -0.75), 1, material_front_right));
    world.add(make_shared<sphere>(point3(0.65, -0.25, -0.25), 0.25, material_small_front_right));
    world.add(make_shared<sphere>(point3(-0.25, -0.25, 0.20), 0.1, material_small_front));
    world.add(make_shared<sphere>(point3(-0.60, -0.45, 0.75), 0.5, material_small_front_right));
    world.add(make_shared<sphere>(point3(-1, 0.0, -2.0), 0.5, material_back_back));
    world.add(make_shared<sphere>(point3(-2, 1.0, -2.0), 0.65, material_back));
    // world.add(make_shared<sphere>(point3(-1.0, -1.0, -1.0), 0.05, ball_color));
    // world.add(make_shared<sphere>(point3(1.0, -1.0, -1.0), 0.05, ball_color));
    // world.add(make_shared<sphere>(point3(0.0, 1, -1.0), 0.05, ball_color));
    // world.add(make_shared<triangle>(point3(-1.0, -1.0, -1.0), point3(1.0, -1.0, -1.0), point3(0, 1, -1.0), material_center));
    // world.add(make_shared<sphere>(point3(-1.0, 0.0, -1.0), 0.5, material_left));
    // world.add(make_shared<sphere>(point3(-1.0, 0.0, -1.0), -0.4, material_left));
    // world.add(make_shared<sphere>(point3(1.0, 0.0, -1.0), 0.5, material_right));
    // unsigned int numFaces = 6;
    // unsigned int faceIndex[6] = {4, 4, 4, 4, 4, 4};
    // unsigned int vertexIndex[24] = {0, 1, 2, 3, 0, 4, 5, 1, 1, 5, 6, 2, 0, 3, 7, 4, 5, 4, 7, 6, 2, 6, 7, 3};
    // vec3 verts[8] = {vec3(-1, 1, 1), vec3(1, 1, 1), vec3(1, 1, -1), vec3(-1, 1, -1), vec3(-1, -1, 1), vec3(1, -1, 1), vec3(1, -1, -1), vec3(-1, -1, -1)};
    // world.add(make_shared<mesh>(numFaces, faceIndex, vertexIndex, verts, material_center));
    
    camera cam;

    // cam.background_colour = colour(0,0,0);
    cam.aspect_ratio = 16.0 / 9.0;
    cam.image_width = 800;
    // cam.image_width = 2000;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;

    cam.vfov = 50;
    // cam.lookfrom = point3(-10, 65, 300);
    cam.lookfrom = point3(0, 0, 5);
    // cam.lookat = point3(-0.5, 0.5, 0.5);
    cam.vup = vec3(0, 1, 0);

    // cam.defocus_angle = 1;
    // cam.focus_dist = 2;
    auto mag_colour = make_shared<lambertian>(colour(0.77734, 0.265625, 0.33984));
    // // world.add(make_shared<sphere>(point3(200, 120, 50.0), 150.5, material_light));
    auto start_parse = high_resolution_clock::now();
    Parser obj_parser;
    obj_parser.parse_obj("../obj_files/example.obj");
    world.add(std::make_shared<mesh>(obj_parser.num_faces, obj_parser.face_index, obj_parser.vertex_index, obj_parser.vertices, mag_colour));


    // Parser obj_parser_2;
    // obj_parser_2.parse_obj("../obj_files/mag.obj");
    // world.add(std::make_shared<mesh>(obj_parser_2.num_faces, obj_parser_2.face_index, obj_parser_2.vertex_index, obj_parser_2.vertices, mag_colour));

    Parser obj_parser_3;
    obj_parser_3.parse_obj("../obj_files/cube3.obj");
    world.add(std::make_shared<mesh>(obj_parser_3.num_faces, obj_parser_3.face_index, obj_parser_3.vertex_index, obj_parser_3.vertices, mag_colour));

    auto stop_parse = high_resolution_clock::now();
    auto duration_parse = duration_cast<seconds>(stop_parse - start_parse);
    clog << "DURATION OF PARSING " << duration_parse.count() << endl;

    //IMPORTANT LINE IF world is a BVH
    world.compute_object_bounds();
    auto start_render = high_resolution_clock::now();
    cam.render(world);
    auto stop_render = high_resolution_clock::now();

    auto duration_render = duration_cast<seconds>(stop_render - start_render);

    clog << "Duration of Render: " << duration_render.count() << endl;;

    return 0;
}