#pragma once 
#include"Shape.h"

namespace pbrt {
	struct TriangleMesh {
		//还是需要把坑先填好
		TriangleMesh(const Transform& ObjectToWorld, int nTriangles, const int* vertexIndices, int nVertices, const Point3f* P, const Vector3f* S, const Normal3f* N, const Point2f* uv, const std::shared_ptr<Texture<float>>& alphaMask);
		const int nTriangles, nVertices;
		std::vector<int> vertexIndices;
		std::unique_ptr<Point3f[]>p;
		std::unique_ptr<Normal3f[]>n;
		std::unique_ptr<Vector3f[]>s;
		std::unique_ptr<Point2f[]>uv;
		std::shared_ptr<Texture<float>>alphaMask;
	};
	class Triangle :public Shape {
	public:
		Triangle(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation, const std::shared_ptr<TriangleMesh>& mesh, int triNumber)
			:Shape(ObjectToWorld, WorldToObject, reverseOrientation), mesh(mesh) {
			v = &mesh->vertexIndices[3 * triNumber];
		}
		Bounds3f ObjectBound()const;
		Bounds3f WorldBound()const;
		bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture)const;
		float Area()const;
		Interaction Sample(const Point2f& u)const;
	private:
		void GetUVs(Point2f uv[3])const {
			if (mesh->uv) {
				uv[0] = mesh->uv[v[0]];
				uv[1] = mesh->uv[v[1]];
				uv[2] = mesh->uv[v[2]];
			}
			else
			{
				uv[0] = Point2f(0, 0);
				uv[1] = Point2f(1, 0);
				uv[2] = Point2f(1, 1);
			}
		}
		std::shared_ptr<TriangleMesh> mesh;
		const int* v;
	};
}