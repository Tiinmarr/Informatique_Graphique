#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI 3.1415
#include <cmath>
#include <string>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <random>
#include <list>

static std::default_random_engine engine(10);
static std::uniform_real_distribution<double>uniform(0, 1);

static inline double sqr(double x) { return x * x; }

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		coord[0] = x;
		coord[1] = y;
		coord[2] = z;
	}
	double& operator[](int i) { return coord[i]; }
	double operator[](int i) const { return coord[i]; }

	Vector& operator+=(const Vector& v) {
		coord[0] += v[0];
		coord[1] += v[1];
		coord[2] += v[2];
		return *this;
	}

	double norm2() const {
		return sqr(coord[0]) + sqr(coord[1]) + sqr(coord[2]);
	}

	Vector normalize() {
		Vector v(coord[0], coord[1], coord[2]);
		coord[0] /= sqrt(v.norm2());
		coord[1] /= sqrt(v.norm2());
		coord[2] /= sqrt(v.norm2());
		return *this; 
	}

	double coord[3];
};

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator-(const Vector& a) {
	return Vector(-a[0], -a[1], -a[2]);
}
Vector operator*(const Vector& a, double b) {
	return Vector(a[0] * b, a[1] * b, a[2] * b);
}
Vector operator*(double a, const Vector& b) {
	return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator*(const Vector& a, const Vector& b) {
	return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}
Vector operator/(const Vector& a, double b) {
	return Vector(a[0]/b, a[1]/b,  a[2]/b);
}

double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1],
		a[2] * b[0] - a[0] * b[2],
		a[0] * b[1] - a[1] * b[0]);
}


Vector random_cos(const Vector &N){
	double r1 = uniform(engine);
	double r2 = uniform(engine);
	double x = cos(2 * M_PI * r1) * sqrt(1 - r2);
	double y = sin(2 * M_PI * r1) * sqrt(1 - r2);
	double z = sqrt(r2);
	Vector T1;
	if (std::abs(N[0]) < std::abs(N[1])) {
		if (std::abs(N[0]) < std::abs(N[2])) T1 = Vector(0, -N[2], N[1]);
		else T1 = Vector(-N[1], N[0], 0);
	}
	else {
		if (std::abs(N[1]) < std::abs(N[2])) T1 = Vector(-N[2], 0, N[0]);
		else T1 = Vector(-N[1], N[0], 0);
	}
	T1.normalize();
	Vector T2 = cross(N, T1);
	return Vector(x * T1 + y * T2 + z * N);
}

class Ray 
{
   public :
   Vector O;
   Vector u;

   Ray() {
      O = Vector(0.0, 0.0, 0.0);
      u = Vector(0.0, 0.0, 0.0);
   }

   Ray(const Vector& OO, const Vector& uu)
   {
      O = OO;
      u = uu;
   }
};

class Geometry {
public:
	Geometry(const Vector& albedo = Vector(1.0, 1.0, 1.0), bool mirror = false, bool transparent = false, bool inverseN = false)
		: albedo(albedo), mirror(mirror), transparent(transparent), inverseN(inverseN) {}

	virtual bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const = 0;

	Vector albedo;
	bool mirror;
	bool transparent;
	bool inverseN;
};


class BoundingBox {
public:
	BoundingBox() {}
	BoundingBox(const Vector& pmin, const Vector& pmax) : pmin(pmin), pmax(pmax) {}
	bool has_intersection(const Ray& r) const {
		Vector inverse(1./r.u[0], 1./r.u[1], 1./r.u[2]);
		double tmin_x = (pmin[0] - r.O[0]) * inverse[0]; // si on divise au lieu de multiplier on obtient un chat invisible :)
		double tmax_x = (pmax[0] - r.O[0]) * inverse[0];
		double entree_x = std::min(tmin_x, tmax_x);
		double sortie_x = std::max(tmin_x, tmax_x);

		double tmin_y = (pmin[1] - r.O[1]) * inverse[1];
		double tmax_y = (pmax[1] - r.O[1]) * inverse[1];
		double entree_y = std::min(tmin_y, tmax_y);
		double sortie_y = std::max(tmin_y, tmax_y);

		double tmin_z = (pmin[2] - r.O[2]) * inverse[2];
		double tmax_z = (pmax[2] - r.O[2]) * inverse[2];
		double entree_z = std::min(tmin_z, tmax_z);
		double sortie_z = std::max(tmin_z, tmax_z);

		double t_entree = std::max(entree_x, std::max(entree_y, entree_z));
		double t_sortie = std::min(sortie_x, std::min(sortie_y, sortie_z));

		if (t_sortie<0) return false;
		if (t_entree > t_sortie) return false;
		return true;
	}

	Vector pmin, pmax;
};

class Node {
public:
	Node(const BoundingBox& bbox,int index_init = -1,int index_end=-1, Node* fg = NULL, Node* fd = NULL) : bbox(bbox), fg(fg), fd(fd), index_init(index_init), index_end(index_end)  {}
	BoundingBox bbox;
	Node* fg;
	Node* fd;
	int index_init;
	int index_end;
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Geometry {
public:
TriangleMesh(const Vector& albedo = Vector(1.0, 1.0, 1.0), bool mirror = false, bool transparent = false, bool inverseN = false)
        : Geometry(albedo, mirror, transparent, inverseN) {}
  ~TriangleMesh() {}
	// TriangleMesh() {};
	// BoundingBox bbox;
	Node* root;
	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				} else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						} else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					} else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							} else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								} else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);

	}

	bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const {
		double new_t =1E10;
		bool has_inter =false;
		if (!root->bbox.has_intersection(r)) return false;
		std::list<Node*> nodes;
		nodes.push_back(root);

		while (!nodes.empty())
		{
			const Node* actual = nodes.back();
			nodes.pop_back();

			if (actual->fg) {
				if (actual->fg->bbox.has_intersection(r)) {
					nodes.push_back(actual->fg);
				}
				if (actual->fd->bbox.has_intersection(r)) {
					nodes.push_back(actual->fd);
				}
			}
			else {
				for (int i = actual->index_init; i < actual->index_end; i++) {
					const Vector& A = vertices[indices[i].vtxi];
					const Vector& B = vertices[indices[i].vtxj];
					const Vector& C = vertices[indices[i].vtxk];

					Vector e1 = B - A;
					Vector e2 = C - A;
					Vector localN = cross(e1, e2);

					Vector OA_u = cross(A - r.O, r.u);

					double uN = dot(r.u, localN);

					double beta = dot(e2, OA_u) / uN;
					double gamma = -dot(e1, OA_u) / uN;
					double t_local = dot(A - r.O, localN) / uN;
					double alpha = 1 - beta - gamma;

					if (beta >= 0 && gamma >= 0 && beta <= 1 && gamma <= 1 && alpha >= 0 && t_local > 0) {
						has_inter = true;
						if (t_local < t) {
							t = t_local;
							P = r.O + t * r.u;
							N = localN;
							N.normalize();
						}
					}
				}
			}
		}
		return has_inter;
		// double new_t =1E10;
		// // bool has_inter = false;
		// bool has_inter;
		// for (int i=0; i<indices.size(); i++)
		// {
		// 	const Vector& A = vertices[indices[i].vtxi];
		// 	const Vector& B = vertices[indices[i].vtxj];
		// 	const Vector& C = vertices[indices[i].vtxk];

		// 	Vector e1 = B - A;
		// 	Vector e2 = C - A;
		// 	Vector localN = cross(e1, e2);

		// 	Vector OA_u = cross(A - r.O, r.u);

		// 	double uN = dot(r.u, localN);

		// 	double beta = dot(e2, OA_u) / uN;
		// 	double gamma = -dot(e1, OA_u) / uN;
		// 	double t_local = dot(A - r.O, localN) / uN;  
		// 	double alpha = 1 - beta - gamma;

		// 	if (beta >= 0 && gamma >= 0 && beta <= 1 && gamma <=1 && alpha >= 0 && t_local >0) {
		// 		has_inter = true;
		// 		if (t_local < new_t) {
		// 			new_t = t_local;
		// 			t = t_local;
		// 			P = r.O + t * r.u;
		// 			N = localN;
		// 			N.normalize();
		// 		}
		// 	}
		// }
		// return has_inter;
	}

	BoundingBox printBox(int triangle_init, int triangle_end) {
		BoundingBox bbox;
		bbox.pmin = Vector(1E10, 1E10, 1E10);
		bbox.pmax = Vector(-1E10, -1E10, -1E10);

		for (int i=triangle_init;i<triangle_end;i++) {
			for (int j=0;j<3;j++) {
			bbox.pmin[j] = std::min(bbox.pmin[j], vertices[indices[i].vtxi][j]);
			bbox.pmin[j] = std::min(bbox.pmin[j], vertices[indices[i].vtxj][j]);
			bbox.pmin[j] = std::min(bbox.pmin[j], vertices[indices[i].vtxk][j]);


			bbox.pmax[j] = std::max(bbox.pmax[j], vertices[indices[i].vtxi][j]);
			bbox.pmax[j] = std::max(bbox.pmax[j], vertices[indices[i].vtxj][j]);
			bbox.pmax[j] = std::max(bbox.pmax[j], vertices[indices[i].vtxk][j]);
			}
		}

		return bbox;
	}

	void print_Tree(Node* node,int init, int end)
	{
		node->index_init = init;
		node->index_end = end;
		BoundingBox b = printBox(init, end);
		node->bbox = b;
		node->fg = NULL;
		node->fd = NULL;

		Vector size = b.pmax - b.pmin;
		Vector mid = (b.pmin + b.pmax) / 2;
		int dimension;
		if (size[0] > size[1] && size[0] > size[2]) dimension = 0;
		else if (size[1] > size[2]) dimension = 1;
		else dimension = 2;

		// Qsort
		int pivot = init;
		for (int i = init; i < end; i++) {
			double mid_triangle = (vertices[indices[i].vtxi][dimension] + vertices[indices[i].vtxj][dimension] + vertices[indices[i].vtxk][dimension]) / 3;
			if (mid_triangle < mid[dimension]) {
				std::swap(indices[i], indices[pivot]);
				pivot++;
			}
		}

		if(end - init < 5 || pivot - init < 2 || end - pivot <2) return;

		node->fg = new Node(b);
		node->fd = new Node(b);
		print_Tree(node->fg, init, pivot);
		print_Tree(node->fd, pivot, end);
	}

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;

	void scale(double s) {
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i] = vertices[i] * s;
		}
	}

	void translate(const Vector& t) {
		for (int i = 0; i < vertices.size(); i++) {
			vertices[i] = vertices[i] + t;
		}
	}

	void rotate(double angle, const Vector& axis) {
		double s = sin(angle);
		double c = cos(angle);
		for (int i = 0; i < vertices.size(); i++) {
			double x = vertices[i][0];
			double y = vertices[i][1];
			double z = vertices[i][2];
			vertices[i][0] = x * (c + sqr(axis[0]) * (1 - c)) + y * (axis[0] * axis[1] * (1 - c) - axis[2] * s) + z * (axis[0] * axis[2] * (1 - c) + axis[1] * s);
			vertices[i][1] = x * (axis[1] * axis[0] * (1 - c) + axis[2] * s) + y * (c + sqr(axis[1]) * (1 - c)) + z * (axis[1] * axis[2] * (1 - c) - axis[0] * s);
			vertices[i][2] = x * (axis[2] * axis[0] * (1 - c) - axis[1] * s) + y * (axis[2] * axis[1] * (1 - c) + axis[0] * s) + z * (c + sqr(axis[2]) * (1 - c));
		}
	}
	
};



class Sphere : public Geometry
{
  public :
  Vector centre;
  float rayon;
//   Vector albedo;
//   bool mirror, transparent, inverseN;

  Sphere() {
    centre = Vector(0.0, 0.0, 0.0);
    rayon = (1.0);
	// albedo = Vector(1.0, 1.0, 1.0);
  }

//   Sphere(const Vector& c, float r, const Vector& alb, bool mir = false, bool trans = false, bool invN = false) {
//     centre= c;
//     rayon = r;
// 	albedo = alb;
// 	mirror = mir;
// 	transparent = trans;
// 	inverseN = invN;
//   }

Sphere(const Vector& c, float r, const Vector& alb, bool mir = false, bool trans = false, bool invN = false) : centre(c), rayon(r), Geometry(alb, mir, trans, invN) {}


  bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const {
	double a = 1;
	double b = 2*dot(r.u,r.O - centre);
	double c = (r.O-centre).norm2() - rayon * rayon;
	double delta = b * b - 4 * a * c;

	if (delta < 0) return false;
	double t1 = (-b - sqrt(delta)) / (2 * a);
	double t2 = (-b + sqrt(delta)) / (2 * a);
	if (t2 < 0) return false; 
	t = t1;
	if (t1 <0) t = t2;
	P = r.O+ t  * r.u;
	N = P - centre;
	N.normalize();

	return true;
  }
};

class Scene
{
  public :
	std::vector<Geometry*> objet;

  Scene() {}

  bool intersect(const Ray& r, Vector& P, Vector& N, int& index, double& best_t){
	bool has_inter = false;
	Vector P_loc, N_loc;
	double t_local;
	best_t = 1E10;
	for (int i = 0; i < objet.size();i++) {
		if (objet[i]->intersect(r, P_loc, N_loc, t_local)) {
			has_inter = true;
			if (t_local < best_t) {
				best_t = t_local;
				index = i;
				P = P_loc;
				if (objet[i]->inverseN) N = -N_loc;
				else N = N_loc;
			}
		}
	}
	return has_inter;
  }

  Vector GetColor(Ray r, int bounce, bool last_bounce_diffuse = false){
	Vector P,N;
	double best_t;
	int index;
	Vector lumiere(-10,20,40);
	double intensity = 1E10;
	Vector lum = dynamic_cast<Sphere*>(objet[0])->centre;
	float radius = dynamic_cast<Sphere*>(objet[0])->rayon;

	if (bounce == 0) return Vector(0,0,0);
	if (intersect(r,P,N,index, best_t)) 
	
	{ 
		if (index == 0) {
			if (last_bounce_diffuse) return Vector(0,0,0);
			else return Vector(1,1,1)* intensity/(4*M_PI * M_PI *  sqr(radius));
		}
		if (objet[index]->mirror) {
			Ray reflectd_ray = Ray(P + 0.01*N, r.u - 2 * dot(r.u,N) * N);
			return GetColor(reflectd_ray, bounce - 1);
		}
		else if (objet[index]->transparent) {
			double n1 = 1;
			double n2 = 1.5;
			if (dot(r.u,N) > 0) {
				std::swap(n1,n2);
				N = -N;
			}
			Vector Tt,Tn;
			Tn = n1/n2 * (r.u - dot(r.u,N) * N);
			double rad = 1 - sqr(n1/n2) * (1 - sqr(dot(r.u,N)));
			if (rad < 0) {
				Vector R = r.u - 2 * dot(r.u,N) * N;
				return GetColor(Ray(P + 0.01*N, R), bounce - 1);
			}
			Tt = -sqrt(rad) * N;
			Vector T = Tn + Tt;
			Ray refracted_ray = Ray(P - 0.01*N, T);


			// si on veut annuler le coeff de fresnel :
			// return GetColor(refracted_ray, bounce - 1);
			
			// calcul coeff de fresnel : 
			Vector col1 = GetColor(refracted_ray, bounce - 1);
			double k0 = sqr((n1 - n2)/ (n1 + n2));
			double R0 = k0 + (1 - k0) * std::pow(1 - std::abs(dot(r.u,N)),5);
			double T0 = 1 - R0;
			Ray reflectd_ray = Ray(P + 0.01*N, r.u - 2 * dot(r.u,N) * N);
			Vector col2 = GetColor(reflectd_ray, bounce - 1);
			Vector col = R0 * col2 + T0 * col1 ;
			return col;
		}
		else {
		Vector col;
		Vector PL = lum - P;
		// double d2 = PL.norm2();
		PL.normalize();
		Vector NPrim = random_cos(-PL);
		Vector PPrim = NPrim*radius + lum;
		Vector PLPrim = PPrim - P; //w_i
		double d2 = PLPrim.norm2();
		PLPrim.normalize();
		// double probability_density = dot(NPrim,-PL) / (M_PI * sqr(objet[0].rayon));

		double L_wi = intensity/(4*M_PI * M_PI *  sqr(radius));
		col = objet[index]->albedo * L_wi * std::max(0.,dot(N,PLPrim)) * std::max(0.,dot(NPrim, -PLPrim)) * sqr(radius) /(d2 * std::max(0.,dot(NPrim,-PL)));

		Ray New_Ray = Ray(P+0.001*N, PLPrim);
		Vector new_P;
		Vector new_N;
		int new_index;
		if (intersect(New_Ray,new_P,new_N,new_index, best_t) && sqr(best_t + 0.01) < d2) 
		{
			col = Vector(0,0,0);
		}
		Vector dir_indirect = random_cos(N);
		Ray r_indirect(P + 0.01 * N, dir_indirect);
		col += objet[index]->albedo * GetColor(r_indirect, bounce - 1,true);
		return col;
		}
	}
	return Vector(0,0,0);
 }

};

int main() {
	int W = 512;
	int H = 512;
	int N_rays = 100;

	TriangleMesh mesh(Vector(1,0.3,0));
	mesh.readOBJ("cat.obj");
	mesh.scale(0.5);
	mesh.translate(Vector(-25,-10,8));
	mesh.rotate(-M_PI/4, Vector(0,1,0));
	mesh.root = new Node(mesh.printBox(0, mesh.indices.size()));
	mesh.print_Tree(mesh.root, 0, mesh.indices.size());

	// TriangleMesh mesh2(Vector(0.,0.,0.));
	// mesh2.readOBJ("cat.obj");
	// mesh2.scale(0.5);
	// mesh2.translate(Vector(-25,-10,-8));
	// mesh2.rotate(-3*M_PI/4, Vector(0,1,0));
	// mesh2.printBox();

	// mesh.readOBJ("car.obj");
	// mesh.scale(1.3);
	// mesh.rotate(M_PI/2, Vector(0,1,0));
	// mesh.translate(Vector(0,-5,0));
	// mesh.printBox();
	

	Scene scene;
	Sphere lumiere(Vector(-10,20,35), 10, Vector(1,1,1)); //light
	Sphere sphere(Vector(0.0,20.0,-10.0), 8.0,Vector(0.3,0.9,0.4), true);//mirror
	Sphere sphere2_1(Vector(-10.0,0.0,25.0), 8.0,Vector(0.3,0.4,0.9),false,true);//sphere_creuse
	Sphere sphere2_2(Vector(-10.0,0.0,25.0), 7.8,Vector(0.3,0.4,0.9),false,true,true);//inversion
	// Sphere sphere3(Vector(-20.0,0.0,0.0), 8.0,Vector(0.7,0.4,0.2),false,true);//transparent
	Sphere sphere3(Vector(0.0,0.0,10.0), 8.0,Vector(0.7,0.4,0.2),false);
	// Sphere sphere_pleine(Vector(10,0,10), 4.0,Vector(0.7,0.4,0.2));
	Sphere green(Vector(0.0,0.0,-1000.), 940.,Vector(0.,1.,0.));
    Sphere red(Vector(0.0,1000.0,0.), 940.,Vector(1.,0.,0.));
	Sphere rose(Vector(0.0,0.0,1000.), 940,Vector(1.,0.,0.5));
	Sphere blue(Vector(0.0,-1000.0,0.), 990,Vector(0.,0.,1.));
	Sphere yellow(Vector(-1000.0,0.0,0.), 940,Vector(1.,1.,0.));
	Sphere magenta(Vector(1000.0,0.0,0.), 940,Vector(0.8,0.,1.));
	scene.objet.push_back(&lumiere);
	scene.objet.push_back(&green);
	scene.objet.push_back(&red);
	scene.objet.push_back(&rose);
	scene.objet.push_back(&blue);
	scene.objet.push_back(&yellow);
	scene.objet.push_back(&magenta);
	scene.objet.push_back(&sphere);
	// scene.objet.push_back(&sphere2_1);
	// scene.objet.push_back(&sphere2_2);
	// scene.objet.push_back(&sphere3);
	// scene.objet.push_back(&sphere_pleine);
	scene.objet.push_back(&mesh);
	// scene.objet.push_back(&mesh2);
 // Scene
	Vector camera(0.0,0.0,55.0);
	double fov = 60 * M_PI / 180;
	double d = W / (2 * tan(fov/2));
	double mise_au_point = 55;
	double ouverture = 1.5;

	Vector center(0.2, 0.1, 0.);
	Vector color;
	std::vector<unsigned char> image(W * H * 3, 0);
	bool blur = false;

	int bounce = 7;
#pragma omp parallel for 
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector col;
			for (int k =0; k<N_rays; k++){
				if (blur) {
				// double r1 = uniform(engine);
				// double r2 = uniform(engine);
				// double g1 = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2);
				// double g2 = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2);
				double g1 = uniform(engine)-0.5;
				double g2 = uniform(engine)-0.5;
				Vector v(j - W/2. + 0.5 +g1, -i + H/2. - 0.5 +g2, - d);
				v.normalize();
				double r3 = uniform(engine);
				double r4 = uniform(engine);
				double g3 = sqrt(-2 * log(r3)) * cos(2 * M_PI * r4) * ouverture;
				double g4 = sqrt(-2 * log(r3)) * sin(2 * M_PI * r4) * ouverture;

				Vector Camera_2 = camera + Vector(g3, g4, 0);
				Vector dir = camera + mise_au_point * v - Camera_2;
				dir.normalize();
				Ray r(camera,dir);
				col += scene.GetColor(r, bounce)/N_rays;
				}
				else {
				double r1 = uniform(engine)-0.5;
				double r2 = uniform(engine)-0.5;
				Vector v(j - W/2. + 0.5 +r1, -i + H/2. - 0.5 +r2, - d);
				v.normalize();
				Ray r(camera,v);
				col += scene.GetColor(r, bounce)/N_rays;
				}
			}
			image[(i * W + j) * 3 + 0] = std::min(255.,std::pow(col[0],0.45));   // RED
			image[(i * W + j) * 3 + 1] = std::min(255.,std::pow(col[1],0.45)) ;  // GREEN
			image[(i * W + j) * 3 + 2] =  std::min(255.,std::pow(col[2],0.45));  // BLUE
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}