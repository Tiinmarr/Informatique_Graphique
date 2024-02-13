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

  Vector GetColor(Ray r, int bounce, bool last_bounce_diffuse = false){
	Vector P,N;
	double best_t;
	int index;
	Vector lumiere(-10,20,40);
	double intensity = 1E10;

	if (bounce == 0) return Vector(0,0,0);
	if (intersect(r,P,N,index, best_t)) 
	
	{ 
		if (index == 0) {
			if (last_bounce_diffuse) return Vector(0,0,0);
			else return Vector(1,1,1)* intensity/(4*M_PI * M_PI *  sqr(objet[0].rayon));
		}
		if (objet[index].mirror) {
			Ray reflectd_ray = Ray(P + 0.01*N, r.u - 2 * dot(r.u,N) * N);
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
		// Vector PL = lumiere - P;
		// double d2 = PL.norm2();
		// PL.normalize();
		// Ray New_Ray = Ray(P+0.01*N, PL);
		// Vector new_P;
		// Vector new_N;
		// int new_index;
		// if (intersect(New_Ray,new_P,new_N,new_index, best_t) && (best_t * best_t) < d2) 
		// {
		// 	Vector col = Vector(0,0,0);
		// 	Vector dir_indirect = random_cos(N);
		// 	Ray r_indirect(P + 0.01 * N, dir_indirect);
		// 	Vector I_indirect = objet[index].albedo * GetColor(r_indirect, bounce - 1);
		// 	col += I_indirect;
		// 	return col;
		// }
		// else {
		// 	Vector col = objet[index].albedo * intensity * std::max(0.,dot(PL,N)) * (1/(4*M_PI*M_PI*d2));
		// 	Vector dir_indirect = random_cos(N);
		// 	Ray r_indirect(P + 0.01 * N, dir_indirect);
		// 	Vector I_indirect = objet[index].albedo * GetColor(r_indirect, bounce - 1);
		// 	col += I_indirect;
		// 	return col;
		// }
		Vector col;
		Vector PL = objet[0].centre - P;
		// double d2 = PL.norm2();
		PL.normalize();
		Vector NPrim = random_cos(-PL);
		Vector PPrim = NPrim*objet[0].rayon + objet[0].centre;
		Vector PLPrim = PPrim - P; //w_i
		double d2 = PLPrim.norm2();
		PLPrim.normalize();
		// double probability_density = dot(NPrim,-PL) / (M_PI * sqr(objet[0].rayon));

		double L_wi = intensity/(4*M_PI * M_PI *  sqr(objet[0].rayon));
		col = objet[index].albedo * L_wi * std::max(0.,dot(N,PLPrim)) * std::max(0.,dot(NPrim, -PLPrim)) * sqr(objet[0].rayon) /(d2 * std::max(0.,dot(NPrim,-PL)));

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
		col += objet[index].albedo * GetColor(r_indirect, bounce - 1,true);
		return col;
		}
	}
	return Vector(0,0,0);
 }

};

int main() {
	int W = 512;
	int H = 512;
	int N_rays = 1500;

	Scene scene;
	Sphere lumiere(Vector(-10,20,40), 10, Vector(1,1,1)); //light
	Sphere sphere(Vector(10.0,0.0,-10.0), 8.0,Vector(0.3,0.9,0.4), true);//mirror
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
	scene.objet.push_back(lumiere);
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
	// scene.objet.push_back(sphere_pleine);
 // Scene
	Vector camera(0.0,0.0,55.0);
	double fov = 60 * M_PI / 180;
	double d = W / (2 * tan(fov/2));
	double mise_au_point = 185;
	double ouverture = 1.5;

	Vector center(0.2, 0.1, 0.);
	Vector color;
	std::vector<unsigned char> image(W * H * 3, 0);
	bool blur = true;

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