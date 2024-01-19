#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define M_PI 3.1415
#include <cmath>

#include <random>
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
// Vector operator/(double a, const Vector& b) {
// 	return Vector(b[0] / a, b[1] / a,  b[2] / a);
// }

double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
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

class Sphere
{
  public :
  Vector centre;
  float rayon;
  Vector albedo;
  bool mirror, transparent, inverseN;

  Sphere() {
    centre = Vector(0.0, 0.0, 0.0);
    rayon = (1.0);
	albedo = Vector(1.0, 1.0, 1.0);

  }

  Sphere(const Vector& c, float r, const Vector& alb, bool mir = false, bool trans = false, bool invN = false) {
    centre= c;
    rayon = r;
	albedo = alb;
	mirror = mir;
	transparent = trans;
	inverseN = invN;
  }

  bool intersect(const Ray& r, Vector& P, Vector& N, double& t){
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
	std::vector<Sphere> objet;

  Scene() {}

  bool intersect(const Ray& r, Vector& P, Vector& N, int& index, double& best_t){
	bool has_inter = false;
	Vector P_loc, N_loc;
	double t_local;
	best_t = 1E10;
	for (int i = 0; i < objet.size();i++) {
		if (objet[i].intersect(r, P_loc, N_loc, t_local)) {
			has_inter = true;
			if (t_local < best_t) {
				best_t = t_local;
				index = i;
				P = P_loc;
				if (objet[i].inverseN) N = -N_loc;
				else N = N_loc;
			}
		}
	}
	return has_inter;
  }

  Vector GetColor(Ray r, int bounce){
	Vector P,N;
	double best_t;
	int index;
	Vector lumiere(-10,20,40);
	double intensity = 1E10;

	if (bounce == 0) return Vector(0,0,0);
	if (intersect(r,P,N,index, best_t)) 
	{ 
		if (objet[index].mirror) {
			Ray reflectd_ray = Ray(P + 0.1*N, r.u - 2 * dot(r.u,N) * N);
			return GetColor(reflectd_ray, bounce - 1);
		}
		else if (objet[index].transparent) {
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
				return GetColor(Ray(P + 0.1*N, R), bounce - 1);
			}
			Tt = -sqrt(rad) * N;
			Vector T = Tn + Tt;
			Ray refracted_ray = Ray(P - 0.1*N, T);


			// si on veut annuler le coeff de fresnel :
			// return GetColor(refracted_ray, bounce - 1);
			
			// calcul coeff de fresnel : 
			Vector col1 = GetColor(refracted_ray, bounce - 1);
			double k0 = sqr((n1 - n2)/ (n1 + n2));
			double R0 = k0 + (1 - k0) * std::pow(1 - std::abs(dot(r.u,N)),5);
			double T0 = 1 - R0;
			Ray reflectd_ray = Ray(P + 0.1*N, r.u - 2 * dot(r.u,N) * N);
			Vector col2 = GetColor(reflectd_ray, bounce - 1);
			Vector col = R0 * col2 + T0 * col1 ;
			return col;
		}
		else {
		Vector PL = lumiere - P;
		double d2 = PL.norm2();
		PL.normalize();
		Ray New_Ray = Ray(P+0.1*N, PL);
		Vector new_P;
		Vector new_N;
		int new_index;
		if (intersect(New_Ray,new_P,new_N,new_index, best_t) && (best_t * best_t) < d2) 
		{
			Vector col = Vector(0,0,0);
			return col;
		}
		else {
			Vector col = objet[index].albedo * intensity * std::max(0.,dot(PL,N)) * (1/(4*M_PI*M_PI*d2));
			return col;
		}
		}
	}
 }

};

int main() {
	int W = 512;
	int H = 512;

	Scene scene;
	Sphere sphere(Vector(0.0,0.0,0.0), 8.0,Vector(0.3,0.9,0.4), true);//mirror
	Sphere sphere2_1(Vector(20.0,0.0,0.0), 8.0,Vector(0.3,0.4,0.9),false,true);//sphere_creuse
	Sphere sphere2_2(Vector(20.0,0.0,0.0), 7.8,Vector(0.3,0.4,0.9),false,true,true);//inversion
	Sphere sphere3(Vector(-20.0,0.0,0.0), 8.0,Vector(0.7,0.4,0.2),false,true);//transparent
	Sphere green(Vector(0.0,0.0,-1000.), 940.,Vector(0.,1.,0.));
    Sphere red(Vector(0.0,1000.0,0.), 940.,Vector(1.,0.,0.));
	Sphere rose(Vector(0.0,0.0,1000.), 940,Vector(1.,0.,0.5));
	Sphere blue(Vector(0.0,-1000.0,0.), 990,Vector(0.,0.,1.));
	Sphere yellow(Vector(-1000.0,0.0,0.), 940,Vector(1.,1.,0.));
	Sphere magenta(Vector(1000.0,0.0,0.), 940,Vector(0.8,0.,1.));
	scene.objet.push_back(green);
	scene.objet.push_back(red);
	scene.objet.push_back(rose);
	scene.objet.push_back(blue);
	scene.objet.push_back(yellow);
	scene.objet.push_back(magenta);
	scene.objet.push_back(sphere);
	scene.objet.push_back(sphere2_1);
	scene.objet.push_back(sphere2_2);
	scene.objet.push_back(sphere3);
 // Scene
	Vector camera(0.0,0.0,55.0);
	double fov = 60 * M_PI / 180;
	double d = W / (2 * tan(fov/2));

	Vector center(0.2, 0.1, 0.);
	Vector color;
	std::vector<unsigned char> image(W * H * 3, 0);

#pragma omp parallel for 
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			double r1 = uniform(engine) - 0.5;
			double r2 = uniform(engine) -0.5;
			Vector v(j - W/2. + 0.5 +r1, -i + H/2. - 0.5 +r2, - d);
			v.normalize();
			Ray r(camera,v);
			int bounce = 5;
			Vector col = scene.GetColor(r, bounce);
			image[(i * W + j) * 3 + 0] = std::min(255.,std::pow(col[0],0.45));   // RED
			image[(i * W + j) * 3 + 1] = std::min(255.,std::pow(col[1],0.45)) ;  // GREEN
			image[(i * W + j) * 3 + 2] =  std::min(255.,std::pow(col[2],0.45));  // BLUE
		}
	}
	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}