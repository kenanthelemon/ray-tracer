#include "ray.hpp"
#include "parser.h"

#include <cmath>
#include <iostream>

Ray::Ray()
{

}

Ray::Ray(parser::Vec3f fromPoint, parser::Vec3f toPoint)
{
    origin = fromPoint;
    direction = toPoint - fromPoint;
    direction.normalize();
}

void ClosestSphere(parser::Scene& scene, Ray& ray, parser::Sphere& sphereCollision, float& t_min_sphere)
{
	for(int i = 0; i < scene.spheres.size(); i++)
	{
		float R = scene.spheres.at(i).radius;
		parser::Vec3f center = scene.vertex_data.at(scene.spheres.at(i).center_vertex_id - 1);
		
		//t = (A +- sqrt(B))/C
		// C = 1 because direction is normalized;
		float B = pow(ray.direction.dotProd(ray.origin - center), 2) - ((ray.origin - center).dotProd(ray.origin - center) - R*R);
		
		//No intersection
		if(B < 0.0f)
			continue;
		
		//Calculate A
		float A = -1*ray.direction.dotProd(ray.origin - center);
		
		//One intersection
		if(B == 0.0f)
		{
			float t = A;
			
			if(t > 0.0f && (t < t_min_sphere || t_min_sphere < 0.0f))
			{
				t_min_sphere = t;
				sphereCollision = scene.spheres.at(i);
			}
		}
		//Two intersections
		else
		{
			float t1 = A + sqrt(B);
			float t2 = A - sqrt(B);
			
			t1 = t1 < t2 ? t1 : t2;
			
			if(t1 > 0.0f && (t1 < t_min_sphere || t_min_sphere < 0.0f))
			{
				t_min_sphere = t1;
				sphereCollision = scene.spheres.at(i);
			}
		}
	}
}

void ClosestTriangle(parser::Scene& scene, Ray& ray, parser::Triangle& triangleCollision, float& t_min_triangle)
{
	for(int i = 0; i < scene.triangles.size(); i++)
	{
		//triangle verteces in counter-clockwise direction
		parser::Vec3f v1, v2, v3;
		
		//Assign triangle verteces
		v1 = scene.vertex_data.at (scene.triangles.at(i).indices.v0_id -1);
		v2 = scene.vertex_data.at (scene.triangles.at(i).indices.v1_id -1);
		v3 = scene.vertex_data.at (scene.triangles.at(i).indices.v2_id -1);
		
		//Determinant of A
		float A = parser::determinant( (v1-v2), (v1-v3), ray.direction);
		
		//Control to avoid division by zero
		if(A == 0.0f)
			continue;
			
		//Barycentric coordinate axis beta
		float beta = parser::determinant( (v1 - ray.origin),
										  (v1 - v3),
										  ray.direction);
	    beta /= A;
		
		//Short-circuit
		if(beta < 0.0f)
			continue;
			
		//Barycentric coordinate axis gamma
		float gamma = parser::determinant( (v1 - v2),
										   (v1 - ray.origin),
										   ray.direction);
	    gamma /= A;
	    
		//Short-circuit
		if(gamma < 0.0f || beta + gamma > 1.0f)
			continue;
			
		//Calculate the t value for the ray.
		float t = parser::determinant( (v1 - v2),
									   (v1 - v3),
									   (v1 - ray.origin));
	    t /= A;
									   
		//Finally, decide if this is a legit intersection
		if(t > 0.0f && (t < t_min_triangle || t_min_triangle < 0.0f))
		{
			t_min_triangle = t;
			triangleCollision = scene.triangles.at(i);
		}
	}
}

void ClosestMesh(parser::Scene& scene, Ray& ray, parser::Mesh& meshCollision, float& t_min_mesh)
{
	for(int i = 0; i < scene.meshes.size(); i++)
	{
		for(int j = 0; j < scene.meshes.at(i).faces.size(); j++)
		{
			//triangle verteces in counter-clockwise direction
			parser::Vec3f v1, v2, v3;
		
			//Assign triangle verteces
			v1 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v0_id -1);
			v2 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v1_id -1);
			v3 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v2_id -1);

			//Determinant of A
			float A = parser::determinant( (v1-v2), (v1-v3), ray.direction);
		
			//Control to avoid division by zero
			if(A == 0.0f)
				continue;
			
			//Barycentric coordinate axis beta
			float beta = parser::determinant( (v1 - ray.origin),
											  (v1 - v3),
											  ray.direction);
			beta /= A;
		
			//Short-circuit
			if(beta < 0.0f)
				continue;
			
			//Barycentric coordinate axis gamma
			float gamma = parser::determinant( (v1 - v2),
											   (v1 - ray.origin),
											   ray.direction);
			gamma /= A;
	    
			//Short-circuit
			if(gamma < 0.0f || beta + gamma > 1.0f)
				continue;
			
			//Calculate the t value for the ray.
			float t = parser::determinant( (v1 - v2),
										   (v1 - v3),
										   (v1 - ray.origin));
			t /= A;
									   
			//Finally, decide if this is a legit intersection
			if(t > 0.0f && (t < t_min_mesh || t_min_mesh < 0.0f))
			{
				t_min_mesh = t;
				meshCollision = scene.meshes.at(i);
				meshCollision.intersectionFace = scene.meshes.at(i).faces.at(j);
			}
		}
	}
}

void determineObjectTypeToRender(int* objectTypeToRender, float& t_min, float& t_min_sphere, float& t_min_triangle, float& t_min_mesh)
{
	if(t_min_sphere > 0.0f) //1xx
	{
		if(t_min_triangle < 0.0f && t_min_mesh < 0.0f) //100
		{
			t_min = t_min_sphere;
			objectTypeToRender[0] = 1;
		}
		else if(t_min_triangle < 0.0f && t_min_mesh > 0.0f) //101
		{
			if(t_min_sphere < t_min_mesh)
			{
				t_min = t_min_sphere;
				objectTypeToRender[0] = 1;
			}
			else
			{
				t_min = t_min_mesh;
				objectTypeToRender[2] = 1;
			}
		}
		else if(t_min_triangle > 0.0f && t_min_mesh < 0.0f) //110
		{
			if(t_min_sphere < t_min_triangle)
			{
				t_min = t_min_sphere;
				objectTypeToRender[0] = 1;
			}
			else
			{
				t_min = t_min_triangle;
				objectTypeToRender[1] = 1;
			}
		}
		else //111
		{
			if(t_min_sphere < t_min_triangle && t_min_sphere < t_min_mesh)
			{
				t_min = t_min_sphere;
				objectTypeToRender[0] = 1;
			}
			else if(t_min_triangle < t_min_sphere && t_min_triangle < t_min_mesh)
			{
				t_min = t_min_triangle;
				objectTypeToRender[1] = 1;
			}
			else
			{
				t_min = t_min_mesh;
				objectTypeToRender[2] = 1;
			}
		}
	}
	else if(t_min_triangle > 0.0f) //01x
	{
		if(t_min_mesh < 0.0f) //010
		{
			t_min = t_min_triangle;
			objectTypeToRender[1] = 1;
		}
		else //011
		{
			if(t_min_triangle < t_min_mesh)
			{
				t_min = t_min_triangle;
				objectTypeToRender[1] = 1;
			}
			else
			{
				t_min = t_min_mesh;
				objectTypeToRender[2] = 1;
			}
		}
	}
	else if(t_min_mesh > 0.0f) //001
	{
		t_min = t_min_mesh;
		objectTypeToRender[2] = 1;
	}
}

bool existsIntersection_Sphere(parser::Scene& scene, Ray& ray, float t_min, float t_max)
{
	for(int i = 0; i < scene.spheres.size(); i++)
	{							
		float R = scene.spheres.at(i).radius;
		parser::Vec3f center = scene.vertex_data.at(scene.spheres.at(i).center_vertex_id - 1);
		float B = pow(ray.direction.dotProd(ray.origin - center), 2) - ((ray.origin - center).dotProd(ray.origin - center) - R*R);
		
		if(B < 0.0f)
			continue;
			
		float A = -1 * ray.direction.dotProd(ray.origin - center);
		
		if(B == 0.0f)
		{
			float t = A;
			
			if (t > t_min && t < t_max)
				return true;
		}
		else
		{
			float t1 = A + sqrt(B);
			
			if (t1 > t_min && t1 < t_max)
				return true;
				
			float t2 = A - sqrt(B);
			
			if (t2 > t_min && t2 < t_max)
				return true;
		}
	}
	
	return false;
}

bool existsIntersection_Triangle(parser::Scene& scene, Ray& ray, float t_min, float t_max)
{
	for(int i = 0; i < scene.triangles.size(); i++)
	{
		//triangle verteces in counter-clockwise direction
		parser::Vec3f v1, v2, v3;
		
		//Assign triangle verteces
		v1 = scene.vertex_data.at (scene.triangles.at(i).indices.v0_id -1);
		v2 = scene.vertex_data.at (scene.triangles.at(i).indices.v1_id -1);
		v3 = scene.vertex_data.at (scene.triangles.at(i).indices.v2_id -1);
		
		//Determinant of A
		float A = parser::determinant( (v1-v2), (v1-v3), ray.direction);
		
		//Control to avoid division by zero
		if(A == 0.0f)
			continue;
			
		//Barycentric coordinate axis beta
		float beta = parser::determinant( (v1 - ray.origin),
										  (v1 - v3),
										  ray.direction);
	    beta /= A;
		
		//Short-circuit
		if(beta < 0.0f)
			continue;
			
		//Barycentric coordinate axis gamma
		float gamma = parser::determinant( (v1 - v2),
										   (v1 - ray.origin),
										   ray.direction);
	    gamma /= A;
	    
		//Short-circuit
		if(gamma < 0.0f || beta + gamma > 1.0f)
			continue;
			
		//Calculate the t value for the ray.
		float t = parser::determinant( (v1 - v2),
									   (v1 - v3),
									   (v1 - ray.origin));
	    t /= A;
									   
		//Finally, decide if this is a legit intersection
		if(t > t_min && t < t_max)
		{
			return true;
		}
	}

	return false;
}

bool existsIntersection_Mesh(parser::Scene& scene, Ray& ray, float t_min, float t_max)
{
	for(int i = 0; i < scene.meshes.size(); i++)
	{
		for(int j = 0; j < scene.meshes.at(i).faces.size(); j++)
		{
			//triangle verteces in counter-clockwise direction
			parser::Vec3f v1, v2, v3;
		
			//Assign triangle verteces
			v1 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v0_id -1);
			v2 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v1_id -1);
			v3 = scene.vertex_data.at (scene.meshes.at(i).faces.at(j).v2_id -1);

			//Determinant of A
			float A = parser::determinant( (v1-v2), (v1-v3), ray.direction);
		
			//Control to avoid division by zero
			if(A == 0.0f)
				continue;
			
			//Barycentric coordinate axis beta
			float beta = parser::determinant( (v1 - ray.origin),
											  (v1 - v3),
											  ray.direction);
			beta /= A;
		
			//Short-circuit
			if(beta < 0.0f)
				continue;
			
			//Barycentric coordinate axis gamma
			float gamma = parser::determinant( (v1 - v2),
											   (v1 - ray.origin),
											   ray.direction);
			gamma /= A;
	    
			//Short-circuit
			if(gamma < 0.0f || beta + gamma > 1.0f)
				continue;
			
			//Calculate the t value for the ray.
			float t = parser::determinant( (v1 - v2),
										   (v1 - v3),
										   (v1 - ray.origin));
			t /= A;
									   
			//Finally, decide if this is a legit intersection
			if(t > t_min && t < t_max)
			{
				return true;
			}
		}
	}

	return false;
}

bool inShadow(parser::Scene& scene, parser::PointLight& light, parser::Vec3f& intersectionPoint)
{
	Ray shadowRay(intersectionPoint, light.position);
	
	float lightDistance = (light.position - intersectionPoint).Magnitude();
	
	return		existsIntersection_Sphere(scene, shadowRay, scene.shadow_ray_epsilon, lightDistance)
			||  existsIntersection_Triangle(scene, shadowRay, scene.shadow_ray_epsilon, lightDistance)
			||  existsIntersection_Mesh(scene, shadowRay, scene.shadow_ray_epsilon, lightDistance);
}

parser::Vec3i FinalColor_Sphere(parser::Scene& scene, Ray& ray, parser::Sphere& sphereCollision, parser::Vec3f& intersectionPoint, int recursionCounter)
{
	//Surface normal
	parser::Vec3f surfaceNormal = (intersectionPoint - scene.vertex_data.at(sphereCollision.center_vertex_id - 1)) * (1/sphereCollision.radius);
	
	//Material
	parser::Material material = scene.materials.at(sphereCollision.material_id - 1);
	
	/* COLORS */
	//Ambient
	parser::Vec3f ambientColor = material.ambient * scene.ambient_light;
	
	//Diffuse + Specular
	parser::Vec3f diffuseColor;
	parser::Vec3f specularColor;
	
	for(int i = 0; i < scene.point_lights.size(); i++)
	{
		parser::PointLight light = scene.point_lights.at(i);
		
		parser::Vec3f fromIntersectionPointToLight = light.position - intersectionPoint;
		float lightDistance = fromIntersectionPointToLight.Magnitude();
		
		fromIntersectionPointToLight.normalize();
		
		//Shadow
		bool m_inShadow = inShadow(scene, light, intersectionPoint);
		
		if(m_inShadow)
			continue;
		
		parser::Vec3f receivedIrradiance = light.intensity * (1 / pow(lightDistance, 2.0f));
		parser::Vec3f halfVector = (fromIntersectionPointToLight - ray.direction).normalize();
		
		diffuseColor = diffuseColor
						+ material.diffuse 
						* fmaxf(0.0f, fromIntersectionPointToLight.dotProd(surfaceNormal))
						* receivedIrradiance;
						
		specularColor = specularColor
						+ material.specular
						* pow( fmaxf(0.0f, surfaceNormal.dotProd(halfVector)) , material.phong_exponent)
						* receivedIrradiance;
	}
	
	//Mirror
	parser::Vec3f mirrorColor;
	parser::Vec3i mirrorColor_int;

	if(material.mirror.x != 0.0f || material.mirror.y != 0.0f || material.mirror.z != 0.0f)
	{
		if(recursionCounter <= scene.max_recursion_depth)
		{
			mirrorColor_int = FinalColor_Mirror(scene, ray, surfaceNormal, material.mirror, intersectionPoint, recursionCounter);
			mirrorColor.x = mirrorColor_int.x;
			mirrorColor.y = mirrorColor_int.y;
			mirrorColor.z = mirrorColor_int.z;
		}
	}
	
	//Final Color
	parser::Vec3f finalColor_float = ambientColor + diffuseColor + specularColor + mirrorColor;
	
	//Clamp
	finalColor_float.x = finalColor_float.x < 0 ? 0 : finalColor_float.x; finalColor_float.x = finalColor_float.x > 255 ? 255 : finalColor_float.x;
	finalColor_float.y = finalColor_float.y < 0 ? 0 : finalColor_float.y; finalColor_float.y = finalColor_float.y > 255 ? 255 : finalColor_float.y;
	finalColor_float.z = finalColor_float.z < 0 ? 0 : finalColor_float.z; finalColor_float.z = finalColor_float.z > 255 ? 255 : finalColor_float.z;
	
	//Round
	parser::Vec3i finalColor;
	finalColor.x = (int) roundf(finalColor_float.x);
	finalColor.y = (int) roundf(finalColor_float.y);
	finalColor.z = (int) roundf(finalColor_float.z);
	
	return finalColor;
}

parser::Vec3i FinalColor_Triangle(parser::Scene& scene, Ray& ray, parser::Triangle& triangleCollision, parser::Vec3f& intersectionPoint, int recursionCounter)
{
	//triangle verteces in counter-clockwise direction
	parser::Vec3f v1, v2, v3;
		
	//Assign triangle verteces
	v1 = scene.vertex_data.at (triangleCollision.indices.v0_id -1);
	v2 = scene.vertex_data.at (triangleCollision.indices.v1_id -1);
	v3 = scene.vertex_data.at (triangleCollision.indices.v2_id -1);

	//Surface normal
	parser::Vec3f surfaceNormal = (v2-v1).crossProd( (v3-v1) ).normalize();
	
	//Material
	parser::Material material = scene.materials.at(triangleCollision.material_id - 1);
	
	/* COLORS */
	//Ambient
	parser::Vec3f ambientColor = material.ambient * scene.ambient_light;
	
	//Diffuse + Specular
	parser::Vec3f diffuseColor;
	parser::Vec3f specularColor;
	
	for(int i = 0; i < scene.point_lights.size(); i++)
	{
		parser::PointLight light = scene.point_lights.at(i);
		
		parser::Vec3f fromIntersectionPointToLight = light.position - intersectionPoint;
		float lightDistance = fromIntersectionPointToLight.Magnitude();
		
		fromIntersectionPointToLight.normalize();
		
		//Shadow
		bool m_inShadow = inShadow(scene, light, intersectionPoint);
		
		if(m_inShadow)
			continue;
		
		parser::Vec3f receivedIrradiance = light.intensity * (1 / pow(lightDistance, 2.0f));
		parser::Vec3f halfVector = (fromIntersectionPointToLight - ray.direction).normalize();
		
		diffuseColor = diffuseColor
						+ material.diffuse 
						* fmaxf(0.0f, fromIntersectionPointToLight.dotProd(surfaceNormal))
						* receivedIrradiance;
						
		specularColor = specularColor
						+ material.specular
						* pow( fmaxf(0.0f, surfaceNormal.dotProd(halfVector)) , material.phong_exponent)
						* receivedIrradiance;
	}
	
	//Mirror
	parser::Vec3f mirrorColor;
	parser::Vec3i mirrorColor_int;

	if(material.mirror.x != 0.0f || material.mirror.y != 0.0f || material.mirror.z != 0.0f)
	{
		if(recursionCounter <= scene.max_recursion_depth)
		{
			mirrorColor_int = FinalColor_Mirror(scene, ray, surfaceNormal, material.mirror, intersectionPoint, recursionCounter);
			mirrorColor.x = mirrorColor_int.x;
			mirrorColor.y = mirrorColor_int.y;
			mirrorColor.z = mirrorColor_int.z;
		}
	}
	
	//Final Color
	parser::Vec3f finalColor_float = ambientColor + diffuseColor + specularColor + mirrorColor;
	
	//Clamp
	finalColor_float.x = finalColor_float.x < 0 ? 0 : finalColor_float.x; finalColor_float.x = finalColor_float.x > 255 ? 255 : finalColor_float.x;
	finalColor_float.y = finalColor_float.y < 0 ? 0 : finalColor_float.y; finalColor_float.y = finalColor_float.y > 255 ? 255 : finalColor_float.y;
	finalColor_float.z = finalColor_float.z < 0 ? 0 : finalColor_float.z; finalColor_float.z = finalColor_float.z > 255 ? 255 : finalColor_float.z;
	
	//Round
	parser::Vec3i finalColor;
	finalColor.x = (int) roundf(finalColor_float.x);
	finalColor.y = (int) roundf(finalColor_float.y);
	finalColor.z = (int) roundf(finalColor_float.z);
	
	return finalColor;
}

parser::Vec3i FinalColor_Mesh(parser::Scene& scene, Ray& ray, parser::Mesh& meshCollision, parser::Vec3f& intersectionPoint, int recursionCounter)
{
	//triangle verteces in counter-clockwise direction
	parser::Vec3f v1, v2, v3;
		
	//Assign triangle verteces
	v1 = scene.vertex_data.at (meshCollision.intersectionFace.v0_id -1);
	v2 = scene.vertex_data.at (meshCollision.intersectionFace.v1_id -1);
	v3 = scene.vertex_data.at (meshCollision.intersectionFace.v2_id -1);

	//Surface normal
	parser::Vec3f surfaceNormal = (v2-v1).crossProd( (v3-v1) ).normalize();
	
	//Material
	parser::Material material = scene.materials.at(meshCollision.material_id - 1);
	
	/* COLORS */
	//Ambient
	parser::Vec3f ambientColor = material.ambient * scene.ambient_light;
	
	//Diffuse + Specular
	parser::Vec3f diffuseColor;
	parser::Vec3f specularColor;
	
	for(int i = 0; i < scene.point_lights.size(); i++)
	{
		parser::PointLight light = scene.point_lights.at(i);
		
		parser::Vec3f fromIntersectionPointToLight = light.position - intersectionPoint;
		float lightDistance = fromIntersectionPointToLight.Magnitude();
		
		fromIntersectionPointToLight.normalize();
		
		//Shadow
		bool m_inShadow = inShadow(scene, light, intersectionPoint);
		
		if(m_inShadow)
			continue;
		
		parser::Vec3f receivedIrradiance = light.intensity * (1 / pow(lightDistance, 2.0f));
		parser::Vec3f halfVector = (fromIntersectionPointToLight - ray.direction).normalize();
		
		diffuseColor = diffuseColor
						+ material.diffuse 
						* fmaxf(0.0f, fromIntersectionPointToLight.dotProd(surfaceNormal))
						* receivedIrradiance;
						
		specularColor = specularColor
						+ material.specular
						* pow( fmaxf(0.0f, surfaceNormal.dotProd(halfVector)) , material.phong_exponent)
						* receivedIrradiance;
	}
	
	//Mirror
	parser::Vec3f mirrorColor;
	parser::Vec3i mirrorColor_int;

	if(material.mirror.x != 0.0f || material.mirror.y != 0.0f || material.mirror.z != 0.0f)
	{
		if(recursionCounter <= scene.max_recursion_depth)
		{
			mirrorColor_int = FinalColor_Mirror(scene, ray, surfaceNormal, material.mirror, intersectionPoint, recursionCounter);
			mirrorColor.x = mirrorColor_int.x;
			mirrorColor.y = mirrorColor_int.y;
			mirrorColor.z = mirrorColor_int.z;
		}
	}
	
	//Final Color
	parser::Vec3f finalColor_float = ambientColor + diffuseColor + specularColor + mirrorColor;
	
	//Clamp
	finalColor_float.x = finalColor_float.x < 0 ? 0 : finalColor_float.x; finalColor_float.x = finalColor_float.x > 255 ? 255 : finalColor_float.x;
	finalColor_float.y = finalColor_float.y < 0 ? 0 : finalColor_float.y; finalColor_float.y = finalColor_float.y > 255 ? 255 : finalColor_float.y;
	finalColor_float.z = finalColor_float.z < 0 ? 0 : finalColor_float.z; finalColor_float.z = finalColor_float.z > 255 ? 255 : finalColor_float.z;
	
	//Round
	parser::Vec3i finalColor;
	finalColor.x = (int) roundf(finalColor_float.x);
	finalColor.y = (int) roundf(finalColor_float.y);
	finalColor.z = (int) roundf(finalColor_float.z);
	
	return finalColor;
}

parser::Vec3i FinalColor_Mirror(parser::Scene& scene, Ray& ray, parser::Vec3f& surfaceNormal, parser::Vec3f& mirrorCoefficient, parser::Vec3f& intersectionPoint, int recursionCounter)
{
	recursionCounter++;

	Ray mirrorRay;

	mirrorRay.origin = intersectionPoint;

	mirrorRay.direction = ray.direction + 2 * surfaceNormal * (surfaceNormal.dotProd(-1 * ray.direction));
	mirrorRay.direction.normalize();

	mirrorRay.origin = mirrorRay.origin + mirrorRay.direction * 0.02f;

	/* ***INTERSECTION*** */
    parser::Vec3f mirrorIntersectionPoint;
	
    //Spheres
    float t_min_sphere = -1.0f;
    parser::Sphere sphereCollision;
                
    ClosestSphere(scene, mirrorRay, sphereCollision, t_min_sphere);
				
				
    //Triangles
    float t_min_triangle = -1.0f;
    parser::Triangle triangleCollision;
                
    ClosestTriangle(scene, mirrorRay, triangleCollision, t_min_triangle);
                
                
    //Meshes
    float t_min_mesh = -1.0f;
    parser::Mesh meshCollision;

    ClosestMesh(scene, mirrorRay, meshCollision, t_min_mesh);


	//Determine which object will be rendered
    //Left to right: Sphere, triangle, mesh. Zero for false, non-zero for true.
    float t_min = -1.0f;
    int objectTypeToRender[3] = {0,0,0};
                
    determineObjectTypeToRender(objectTypeToRender, t_min, t_min_sphere, t_min_triangle, t_min_mesh);
                
    //Calculate Intersection Point
    mirrorIntersectionPoint = mirrorRay.origin + t_min * mirrorRay.direction;

	 /* ***SHADING*** */
                
    //Sphere
    if(objectTypeToRender[0])
    {
		parser::Vec3i finalColor = FinalColor_Sphere(scene, mirrorRay, sphereCollision, mirrorIntersectionPoint, recursionCounter);

		finalColor.x = (int) round(finalColor.x * mirrorCoefficient.x);
		finalColor.y = (int) round(finalColor.y * mirrorCoefficient.y);
		finalColor.z = (int) round(finalColor.z * mirrorCoefficient.z);
		
		return finalColor;
	}
				
	//Triangle
	else if(objectTypeToRender[1])
	{
		parser::Vec3i finalColor = FinalColor_Triangle(scene, mirrorRay, triangleCollision, mirrorIntersectionPoint, recursionCounter);

		finalColor.x = (int) round(finalColor.x * mirrorCoefficient.x);
		finalColor.y = (int) round(finalColor.y * mirrorCoefficient.y);
		finalColor.z = (int) round(finalColor.z * mirrorCoefficient.z);

		return finalColor;
	}
				
	//Mesh
	else if(objectTypeToRender[2])
	{
		parser::Vec3i finalColor = FinalColor_Mesh(scene, mirrorRay, meshCollision, mirrorIntersectionPoint, recursionCounter);

		finalColor.x = (int) round(finalColor.x * mirrorCoefficient.x);
		finalColor.y = (int) round(finalColor.y * mirrorCoefficient.y);
		finalColor.z = (int) round(finalColor.z * mirrorCoefficient.z);

		return finalColor;
	}
				
	//There is no intersection
	else
	{
		parser::Vec3i finalColor;

		finalColor = scene.background_color;

		finalColor.x = (int) round(finalColor.x * mirrorCoefficient.x);
		finalColor.y = (int) round(finalColor.y * mirrorCoefficient.y);
		finalColor.z = (int) round(finalColor.z * mirrorCoefficient.z);

		return finalColor;
	}
}