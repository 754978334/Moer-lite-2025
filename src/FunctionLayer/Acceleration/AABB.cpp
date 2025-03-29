#include "AABB.h"

Point3f minP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::min(p1[0], p2[0]), std::min(p1[1], p2[1]),
                 std::min(p1[2], p2[2])};
}

Point3f maxP(const Point3f &p1, const Point3f &p2) {
  return Point3f{std::max(p1[0], p2[0]), std::max(p1[1], p2[1]),
                 std::max(p1[2], p2[2])};
}

AABB AABB::Union(const AABB &other) const {
  Point3f min = minP(other.pMin, pMin), max = maxP(other.pMax, pMax);
  return AABB{min, max};
}

void AABB::Expand(const AABB &other) {
  pMin = minP(pMin, other.pMin);
  pMax = maxP(pMax, other.pMax);
}

AABB AABB::Union(const Point3f &other) const {
  Point3f min = minP(other, pMin), max = maxP(other, pMax);
  return AABB{min, max};
}

void AABB::Expand(const Point3f &other) {
  pMin = minP(pMin, other);
  pMax = maxP(pMax, other);
}

bool AABB::Overlap(const AABB &other) const {
  for (int dim = 0; dim < 3; ++dim) {
    if (pMin[dim] > other.pMax[dim] || pMax[dim] < other.pMin[dim]) {
      return false;
    }
  }
  return true;
}

bool AABB::RayIntersect(const Ray &ray, float *tMin, float *tMax) const {
  //* todo 实现AABB与光线求交
  float _tMin = 0.0f;
  float _tMax = std::numeric_limits<float>::max();

  // X Axis
  for(int i = 0; i < 3; ++i){
    if(ray.direction[i] != 0.0f) {
      float t0 = (pMin[i] - ray.origin[i]) / ray.direction[i];
      float t1 = (pMax[i] - ray.origin[i]) / ray.direction[i];
      if(t0 > t1) std::swap(t0, t1);    // make sure t0 < t1
      _tMin = std::max(_tMin, t0);
      _tMax = std::min(_tMax, t1);
    }
  }
  if(_tMax < _tMin) return false;
  
  if(tMin)*tMin = _tMin;
  if(tMax)*tMax = _tMax;

  return true;
}

Point3f AABB::Center() const {
  return Point3f{(pMin[0] + pMax[0]) * .5f, (pMin[1] + pMax[1]) * .5f,
                 (pMin[2] + pMax[2]) * .5f};
}

size_t AABB::MaxDimension() const {
  size_t dim = 0;
  for (int i = 1; i < 3; ++i) {
    if (pMax[i] - pMin[i] > pMax[dim] - pMin[dim]) {
      dim = i;
    }
  }
  return dim;
}

float AABB::SurfaceArea() const {
  Vector3f d = pMax - pMin;
  return 2 * (d[0] * d[1] + d[1] * d[2] + d[2] * d[0]);
}

float AABB::Volume() const {
  Vector3f d = pMax - pMin;
  return d[0] * d[1] * d[2];
}