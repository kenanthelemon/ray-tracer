#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        public:

            float x, y, z;
            //Constructor
            Vec3f();
            Vec3f(float xCo, float yCo, float zCo);

            //Two-Vector Operations
            Vec3f operator+(const Vec3f& rhs);
            Vec3f operator-(const Vec3f& rhs);
            Vec3f operator=(const Vec3f& rhs);
            Vec3f operator*(const Vec3f& rhs) const;
            float dotProd(const Vec3f& rhs);
            Vec3f crossProd(const Vec3f& rhs) const;

            //Normalize
            Vec3f normalize();

            //Calculate Magnitude
            float Magnitude();
    };

    Vec3f operator*(float lhs, const Vec3f& rhs);
    Vec3f operator*(const Vec3f& lhs, float rhs);
	
	//3x3 dimensional matrix determinant
	float determinant(const Vec3f& vec1, const Vec3f& vec2, const Vec3f& vec3);

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
		Face intersectionFace;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string& filepath);
    };
}

#endif
