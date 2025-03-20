#include "Triangle.h"
#include <FunctionLayer/Acceleration/Linear.h>
//--- Triangle ---
Triangle::Triangle(int _primID, int _vtx0Idx, int _vtx1Idx, int _vtx2Idx,
                   const TriangleMesh *_mesh)
    : primID(_primID), vtx0Idx(_vtx0Idx), vtx1Idx(_vtx1Idx), vtx2Idx(_vtx2Idx),
      mesh(_mesh) {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
          vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
          vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);
  boundingBox.Expand(vtx0);
  boundingBox.Expand(vtx1);
  boundingBox.Expand(vtx2);
  this->geometryID = mesh->geometryID;
}

bool Triangle::rayIntersectShape(Ray &ray, int *primID, float *u,
                                 float *v) const {
  Point3f vtx0 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx0Idx]),
  vtx1 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx1Idx]),
  vtx2 = mesh->transform.toWorld(mesh->meshData->vertexBuffer[vtx2Idx]);

  float eps = std::numeric_limits<float>::epsilon();

  auto S = ray.origin - vtx0;
  auto E1 = vtx1 - vtx0;
  auto E2 = vtx2 - vtx0;
  auto S1 = cross(ray.direction, E2);
  auto S2 = cross(S, E1);

  float S1E1 = dot(S1, E1);
  if(std::abs(S1E1) < eps)
      return false;

  float t = dot(S2, E2) / S1E1;
  float b1 = dot(S1, S) / S1E1;
  float b2 = dot(S2, ray.direction) / S1E1;

  if(t > eps && b1 >= 0.f && b2 >= 0.f && (1 - b1 - b2) >= 0.f)
  {
    if(t < ray.tFar){
      ray.tFar = t;
      *primID = this->primID;
      *u = b1;
      *v = b2;
      return true;
    }
  }

  return false;
}

void Triangle::fillIntersection(float distance, int primID, float u, float v,
                                Intersection *intersection) const {
  // 该函数实际上不会被调用
  return;
}

//--- TriangleMesh ---
TriangleMesh::TriangleMesh(const Json &json) : Shape(json) {
  const auto &filepath = fetchRequired<std::string>(json, "file");
  meshData = MeshData::loadFromFile(filepath);
}

RTCGeometry TriangleMesh::getEmbreeGeometry(RTCDevice device) const {
  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  float *vertexBuffer = (float *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float),
      meshData->vertexCount);
  for (int i = 0; i < meshData->vertexCount; ++i) {
    Point3f vertex = transform.toWorld(meshData->vertexBuffer[i]);
    vertexBuffer[3 * i] = vertex[0];
    vertexBuffer[3 * i + 1] = vertex[1];
    vertexBuffer[3 * i + 2] = vertex[2];
  }

  unsigned *indexBuffer = (unsigned *)rtcSetNewGeometryBuffer(
      geometry, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3,
      3 * sizeof(unsigned), meshData->faceCount);
  for (int i = 0; i < meshData->faceCount; ++i) {
    indexBuffer[i * 3] = meshData->faceBuffer[i][0].vertexIndex;
    indexBuffer[i * 3 + 1] = meshData->faceBuffer[i][1].vertexIndex;
    indexBuffer[i * 3 + 2] = meshData->faceBuffer[i][2].vertexIndex;
  }
  rtcCommitGeometry(geometry);
  return geometry;
}

bool TriangleMesh::rayIntersectShape(Ray &ray, int *primID, float *u,
                                     float *v) const {
  //* 当使用embree加速时，该方法不会被调用
  int geomID = -1;
  return acceleration->rayIntersect(ray, &geomID, primID, u, v);
}

void TriangleMesh::fillIntersection(float distance, int primID, float u,
                                    float v, Intersection *intersection) const {
  
  intersection->distance = distance;
  intersection->shape = this;
  std::array<DataIndex, 3> vtxIdxs = meshData->faceBuffer[primID];
  //* 1. 在三角形内部用插值计算交点坐标
  Point3f p0 = transform.toWorld(meshData->vertexBuffer[vtxIdxs[0].vertexIndex]);
  Point3f p1 = transform.toWorld(meshData->vertexBuffer[vtxIdxs[1].vertexIndex]);
  Point3f p2 = transform.toWorld(meshData->vertexBuffer[vtxIdxs[2].vertexIndex]);
  intersection->position = {
    p0[0] * (1 - u - v) + p1[0] * u + p2[0] * v,
    p0[1] * (1 - u - v) + p1[1] * u + p2[1] * v,
    p0[2] * (1 - u - v) + p1[2] * u + p2[2] * v
  };
  //* 2. 在三角形内部用插值计算法线
  Vector3f p0n = transform.toWorld(meshData->normalBuffer[vtxIdxs[0].normalIndex]);
  Vector3f p1n = transform.toWorld(meshData->normalBuffer[vtxIdxs[1].normalIndex]);
  Vector3f p2n = transform.toWorld(meshData->normalBuffer[vtxIdxs[2].normalIndex]);
  intersection->normal = {
    p0n[0] * (1 - u - v) + p1n[0] * u + p2n[0] * v,
    p0n[1] * (1 - u - v) + p1n[1] * u + p2n[1] * v,
    p0n[2] * (1 - u - v) + p1n[2] * u + p2n[2] * v,
  };
  //* 3. 在三角形内部用插值计算纹理坐标
  Vector2f p0t = meshData->texcodBuffer[vtxIdxs[0].texcodIndex];
  Vector2f p1t = meshData->texcodBuffer[vtxIdxs[1].texcodIndex];
  Vector2f p2t = meshData->texcodBuffer[vtxIdxs[2].texcodIndex];
  intersection->texCoord = {
    p0t[0] * (1 - u - v) + p1t[0] * u + p2t[0] * v,
    p0t[1] * (1 - u - v) + p1t[1] * u + p2t[1] * v,
    p0t[2] * (1 - u - v) + p1t[2] * u + p2t[2] * v,
  };
  //* 4. 在三角形内部用插值计算交点的切线和副切线
  Vector3f e1 = p1 - p0;
  Vector3f e2 = p2 - p0;
  Vector2f duv1 = p1t - p0t;
  Vector2f duv2 = p2t - p0t;

  float f = 1.0f / (duv1.x() * duv2.y() - duv1.y() * duv2.x());
  intersection->tangent = {
    f * (duv2.y() * e1[0] - duv1.y() * e2[0]),
    f * (duv2.y() * e1[1] - duv1.y() * e2[1]),
    f * (duv2.y() * e1[2] - duv1.y() * e2[2])
  };
  intersection->bitangent = {
    f * (-duv2.x() * e1[0] + duv1.x() * e2[0]),
    f * (-duv2.x() * e1[1] + duv1.x() * e2[1]),
    f * (-duv2.x() * e1[2] + duv1.x() * e2[2]),
  };
}

void TriangleMesh::initInternalAcceleration() {
  acceleration = Acceleration::createAcceleration();
  int primCount = meshData->faceCount;
  for (int primID = 0; primID < primCount; ++primID) {
    int vtx0Idx = meshData->faceBuffer[primID][0].vertexIndex,
        vtx1Idx = meshData->faceBuffer[primID][1].vertexIndex,
        vtx2Idx = meshData->faceBuffer[primID][2].vertexIndex;
    std::shared_ptr<Triangle> triangle =
        std::make_shared<Triangle>(primID, vtx0Idx, vtx1Idx, vtx2Idx, this);
    acceleration->attachShape(triangle);
  }
  acceleration->build();
  // TriangleMesh的包围盒就是其内部加速结构的包围盒
  boundingBox = acceleration->boundingBox;
}
REGISTER_CLASS(TriangleMesh, "triangle")