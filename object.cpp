#include "object.hpp"

#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>


bool Object::intersect(Ray ray, Intersection& hit) const
{
    // Assure une valeur correcte pour la coordonnée W de l'origine et de la direction
    // Vous pouvez commentez ces lignes si vous faites très attention à la façon de construire vos rayons.
    ray.origin[3] = 1;
    ray.direction[3] = 0;

    Ray local_ray(i_transform * ray.origin, i_transform * ray.direction);
    //!!! NOTE UTILE : pour calculer la profondeur dans localIntersect(), si l'intersection se passe à
    // ray.origin + ray.direction * t, alors t est la profondeur
    //!!! NOTE UTILE : ici, la direction peut êytre mise à l'échelle, alors vous devez la renormaliser
    // dans localIntersect(), ou vous aurez une profondeur dans le système de coordonnées local, qui
    // ne pourra pas être comparée aux intersection avec les autres objets.
    if (localIntersect(local_ray, hit))
    {
        // Assure la valeur correcte de W.
        hit.position[3] = 1;
        hit.normal[3] = 0;

        // Transforme les coordonnées de l'intersection dans le repère global.
        hit.position = transform * hit.position;
        hit.normal = (n_transform * hit.normal).normalized();

        return true;
    }

    return false;
}

double quadratic(double a, double b, double c) {
    return (pow(b, 2)) - (4 * a * c);
}

bool Sphere::localIntersect(Ray const& ray, Intersection& hit) const
{
    // @@@@@@ VOTRE CODE ICI
    // Vous pourriez aussi utiliser des relations géométriques pures plutôt que les
    // outils analytiques présentés dans les slides.
    // Ici, dans le système de coordonées local, la sphère est centrée en (0, 0, 0)
    //
    // NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
    // donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.


    //variables pour l'équation quadratique
    double a = pow(ray.direction[0], 2) + 
        pow(ray.direction[1], 2) + pow(ray.direction[2], 2);
    double b = 2 * ((ray.origin[0] * ray.direction[0]) + 
        (ray.origin[1] * ray.direction[1]) + (ray.origin[2] * ray.direction[2]));
    double c = (pow(ray.origin[0], 2) + pow(ray.origin[1], 2) + 
        pow(ray.origin[2], 2)) - (pow(this->radius, 2));
    double quad = quadratic(a, b, c);

    double v;

    bool hitInside = false;

    //aucune solution
    if (quad < 0) {
        return false;
    }
    //une solution
    else if (quad == 0) {
        if ((-b) / (2 * a) >= 0) {
            v = (-b) / (2 * a);
        }
        else {
            return false;
        }
    }
    //deux solutions (quad>0)
    else {
        double alpha = (-b + sqrt(quad)) / (2 * a);
        double beta = (-b - sqrt(quad)) / (2 * a);
        if (alpha >= 0 && beta >= 0) {
            if (alpha < beta) {
                v = alpha;
            }
            else {
                v = beta;
            }
        }
        else if (alpha >= 0 && beta < 0) {
            hitInside = true;
            v = alpha;
        }
        else if (beta >= 0 && alpha < 0) {
            hitInside = true;
            v = beta;
        }
        else {
            return false;
        }
    }
    if (hit.depth > v) {
        double x = ray.origin[0] + ray.direction[0] * v;
        double y = ray.origin[1] + ray.direction[1] * v;
        double z = ray.origin[2] + ray.direction[2] * v;
        Vector position = Vector(x, y, z);
        hit.position = position;
        hit.depth = v;

        if (hitInside) {
            hit.normal = -position.normalized();
        }
        else {
            hit.normal = position.normalized();
        }
        return true;
    }
    else {
        return false;
    }
}


bool Plane::localIntersect(Ray const& ray, Intersection& hit) const
{
    // @@@@@@ VOTRE CODE ICI
    // N'acceptez pas les intersections tant que le rayon est à l'intérieur du plan.
    // ici, dans le système de coordonées local, le plan est à z = 0.
    //
    // NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
    // donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.


    double v = -ray.origin[2] / ray.direction[2];
    double x = ray.origin[0] + ray.direction[0] * v;
    double y = ray.origin[1] + ray.direction[1] * v;
    double z = ray.origin[2] + ray.direction[2] * v;

    if (hit.depth > v && v >= 0) {

        Vector position = Vector(x, y, z);
        hit.normal = Vector(0, 0, 1, 0); //la normale du plan est toujour z = 1
        hit.depth = v;
        hit.position = position;
        return true;
    }
    else {
        return false;
    }
}


bool Conic::localIntersect(Ray const& ray, Intersection& hit) const {
    // @@@@@@ VOTRE CODE ICI (licence créative)
    return false;
}


// Intersections !
bool Mesh::localIntersect(Ray const& ray, Intersection& hit) const
{
    // Test de la boite englobante
    double tNear = -DBL_MAX, tFar = DBL_MAX;
    for (int i = 0; i < 3; i++) {
        if (ray.direction[i] == 0.0) {
            if (ray.origin[i] < bboxMin[i] || ray.origin[i] > bboxMax[i]) {
                // Rayon parallèle à un plan de la boite englobante et en dehors de la boite
                return false;
            }
            // Rayon parallèle à un plan de la boite et dans la boite: on continue
        }
        else {
            double t1 = (bboxMin[i] - ray.origin[i]) / ray.direction[i];
            double t2 = (bboxMax[i] - ray.origin[i]) / ray.direction[i];
            if (t1 > t2) std::swap(t1, t2); // Assure t1 <= t2

            if (t1 > tNear) tNear = t1; // On veut le plus lointain tNear.
            if (t2 < tFar) tFar = t2; // On veut le plus proche tFar.

            if (tNear > tFar) return false; // Le rayon rate la boite englobante.
            if (tFar < 0) return false; // La boite englobante est derrière le rayon.
        }
    }
    // Si on arrive jusqu'ici, c'est que le rayon a intersecté la boite englobante.

    // Le rayon interesecte la boite englobante, donc on teste chaque triangle.
    bool isHit = false;
    for (size_t tri_i = 0; tri_i < triangles.size(); tri_i++) {
        Triangle const& tri = triangles[tri_i];

        if (intersectTriangle(ray, tri, hit)) {
            isHit = true;
        }
    }
    return isHit;
}

double Mesh::implicitLineEquation(double p_x, double p_y,
    double e1_x, double e1_y,
    double e2_x, double e2_y) const
{
    return (e2_y - e1_y) * (p_x - e1_x) - (e2_x - e1_x) * (p_y - e1_y);
}

bool Mesh::intersectTriangle(Ray const& ray,
    Triangle const& tri,
    Intersection& hit) const
{
    // Extrait chaque position de sommet des données du maillage.
    Vector const& p0 = positions[tri[0].pi];
    Vector const& p1 = positions[tri[1].pi];
    Vector const& p2 = positions[tri[2].pi];

    // @@@@@@ VOTRE CODE ICI
    // Décidez si le rayon intersecte le triangle (p0,p1,p2).
    // Si c'est le cas, remplissez la structure hit avec les informations
    // de l'intersection et renvoyez true.
    // Vous pourriez trouver utile d'utiliser la routine implicitLineEquation()
    // pour calculer le résultat de l'équation de ligne implicite en 2D.
    //
    // NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
    // donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.
    //!!! NOTE UTILE : pour le point d'intersection, sa normale doit satisfaire hit.normal.dot(ray.direction) < 0

    Vector p01 = p1 - p0;
    Vector p02 = p2 - p0;
    Vector p12 = p2 - p1;
    Vector p20 = p0 - p2;
    Vector normal = (p01.cross(p02)).normalized();//normal ok?

    double v = (((-normal.dot(ray.origin)) + normal.dot(p0)) / normal.dot(ray.direction));
    double x = ray.origin[0] + ray.direction[0] * v;
    double y = ray.origin[1] + ray.direction[1] * v;
    double z = ray.origin[2] + ray.direction[2] * v;

    Vector position = Vector(x, y, z);

    if (  //conditions quand ray ne hit pas
        normal.dot(p01.cross(position - p0)) < 0 ||
        normal.dot(p12.cross(position - p1)) < 0 ||
        normal.dot(p20.cross(position - p2)) < 0 ||
        hit.depth <= v ||
        v < 0)
    {
        return false;
    }
    else {
        hit.position = position;
        hit.normal = normal;
        hit.depth = v;
        return true;
    }
}