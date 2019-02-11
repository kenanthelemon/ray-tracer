#ifndef _RAY_H_
#define _RAY_H_

#include "parser.h"

struct Ray
{
    public:

        parser::Vec3f origin, direction;

        //Constructor
        Ray();
        Ray(parser::Vec3f fromPoint, parser::Vec3f toPoint);
};

//Given a ray, find the closest intersection
void ClosestSphere(parser::Scene& scene, Ray& ray, parser::Sphere& sphereCollision, float& t_min_sphere);

void ClosestTriangle(parser::Scene& scene, Ray& ray, parser::Triangle& triangleCollision, float& t_min_triangle);

void ClosestMesh(parser::Scene& scene, Ray& ray, parser::Mesh& meshCollision, float& t_min_mesh);

//Determine which type of object to be rendered
void determineObjectTypeToRender(int* objectTypeToRender, float& t_min, float& t_min_sphere, float& t_min_triangle, float& t_min_mesh);

//Given a ray and t_min, t_max, determine if there is any intersection
bool existsIntersection_Sphere(parser::Scene& scene, Ray& ray, float t_min, float t_max);

bool existsIntersection_Triangle(parser::Scene& scene, Ray& ray, float t_min, float t_max);

bool existsIntersection_Mesh(parser::Scene& scene, Ray& ray, float t_min, float t_max);


//Gives the final color values between [0,255]
parser::Vec3i FinalColor_Sphere(parser::Scene& scene, Ray& ray, parser::Sphere& sphereCollision, parser::Vec3f& intersectionPoint, int recursionCounter);

parser::Vec3i FinalColor_Triangle(parser::Scene& scene, Ray& ray, parser::Triangle& triangleCollision, parser::Vec3f& intersectionPoint, int recursionCounter);

parser::Vec3i FinalColor_Mesh(parser::Scene& scene, Ray& ray, parser::Mesh& meshCollision, parser::Vec3f& intersectionPoint, int recursionCounter);

//Checks if the intersection point is in a shadow
bool inShadow(parser::Scene& scene, parser::PointLight& light, parser::Vec3f& intersectionPoint);

//Mirror Calculations
parser::Vec3i FinalColor_Mirror(parser::Scene& scene, Ray& ray, parser::Vec3f& surfaceNormal, parser::Vec3f& mirrorCoefficient, parser::Vec3f& intersectionPoint, int recursionCounter);

#endif // _RAY_H_
