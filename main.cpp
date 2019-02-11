#include <iostream>
#include "parser.h"
#include "ppm.h"

#include "ray.hpp"
#include <cmath>

typedef unsigned char RGB[3];

int main(int argc, char* argv[])
{
    parser::Scene scene;
    scene.loadFromXml(argv[1]);

    const unsigned int number_of_cameras = scene.cameras.size();

    for (unsigned int cameraIndex = 0; cameraIndex < number_of_cameras; cameraIndex++)
    {
        int width = scene.cameras.at(cameraIndex).image_width;
        int height = scene.cameras.at(cameraIndex).image_height;
        unsigned char* image = new unsigned char[width * height * 3];

        const RGB bgColor =
        {
            scene.background_color.x,
            scene.background_color.y,
            scene.background_color.z
        };

        //Camera Vectors u,v,w
        parser::Vec3f u,v,w;

        v = scene.cameras.at(cameraIndex).up;
        w = -1 * scene.cameras.at(cameraIndex).gaze;
        u = v.crossProd(w);

        //Image Plane Top Left Corner
        parser::Vec3f q;

        q = scene.cameras.at(cameraIndex).position
                - w * scene.cameras.at(cameraIndex).near_distance
                + u * scene.cameras.at(cameraIndex).near_plane.x
                + v * scene.cameras.at(cameraIndex).near_plane.w;

        //Other constants for ray calculations
        float k_su = (scene.cameras.at(cameraIndex).near_plane.y
               - scene.cameras.at(cameraIndex).near_plane.x)
               / width;
        float k_sv = (scene.cameras.at(cameraIndex).near_plane.w
               - scene.cameras.at(cameraIndex).near_plane.z)
               / height;
        
	    int imageIndex = 0;

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
				parser::Vec3f intersectionPoint;
				
                //Calculate Ray
                parser::Vec3f s;
                s = q + (x + 0.5f) * k_su * u - (y + 0.5f) * k_sv * v;

                Ray ray(scene.cameras.at(cameraIndex).position, s);
                
                /* ***INTERSECTION*** */
                
                //Spheres
                float t_min_sphere = -1.0f;
                parser::Sphere sphereCollision;
                
                ClosestSphere(scene, ray, sphereCollision, t_min_sphere);
				
				
                //Triangles
                float t_min_triangle = -1.0f;
                parser::Triangle triangleCollision;
                
                ClosestTriangle(scene, ray, triangleCollision, t_min_triangle);
                
                
                //Meshes
                float t_min_mesh = -1.0f;
                parser::Mesh meshCollision;

                ClosestMesh(scene, ray, meshCollision, t_min_mesh);
                
                
                //Determine which object will be rendered
                //Left to right: Sphere, triangle, mesh. Zero for false, non-zero for true.
                float t_min = -1.0f;
                int objectTypeToRender[3] = {0,0,0};
                
                determineObjectTypeToRender(objectTypeToRender, t_min, t_min_sphere, t_min_triangle, t_min_mesh);
                
                //Calculate Intersection Point
                intersectionPoint = ray.origin + t_min * ray.direction;
                
                /* ***SHADING*** */
                
                //Sphere
                if(objectTypeToRender[0])
                {
					parser::Vec3i finalColor = FinalColor_Sphere(scene, ray, sphereCollision, intersectionPoint, 1);
					
					image[imageIndex++] = finalColor.x;
					image[imageIndex++] = finalColor.y;
					image[imageIndex++] = finalColor.z;
				}
				
				//Triangle
				else if(objectTypeToRender[1])
				{
					parser::Vec3i finalColor = FinalColor_Triangle(scene, ray, triangleCollision, intersectionPoint, 1);

					image[imageIndex++] = finalColor.x;
					image[imageIndex++] = finalColor.y;
					image[imageIndex++] = finalColor.z;
				}
				
				//Mesh
				else if(objectTypeToRender[2])
				{
					parser::Vec3i finalColor = FinalColor_Mesh(scene, ray, meshCollision, intersectionPoint, 1);

					image[imageIndex++] = finalColor.x;
					image[imageIndex++] = finalColor.y;
					image[imageIndex++] = finalColor.z;
				}
				
				//There is no intersection
				else
				{
					image[imageIndex++] = bgColor[0];
					image[imageIndex++] = bgColor[1];
					image[imageIndex++] = bgColor[2];
				}
            }
        }
		const char * imageName = scene.cameras.at(cameraIndex).image_name.c_str();
        write_ppm(imageName, image, width, height);
    }
}








