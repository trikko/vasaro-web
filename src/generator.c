#include "generator.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "raylib.h"
#include "raymath.h"
#include "hashmap.h"
#include "noises/simplexnoise.h"

extern Model   model;
extern Shader  shader;
extern Vase    vase;

// Calculate spline coeff. from ten points equally-separated
void naturalSpline(int32_t yy[10], float results[40])
{
   float mu[9];
   float z[10];
   float g = 0;
   float y[10];

   for(size_t i = 0; i < 10; ++i)
      y[i] = yy[i]/1000.0f;

   mu[0] = 0;
   z[0] = 0;

   for (int i = 1; i < 9; i++) {
      g = 4 - mu[i -1];
      mu[i] = 1 / g;
      z[i] = (3 * (y[i + 1]  - y[i] * 2 + y[i - 1]) - z[i - 1]) / g;
   }

   float b[9];
   float c[10];
   float d[9];

   z[9] = 0;
   c[9] = 0;

   for (int j = 8; j >=0; j--) {
      c[j] = z[j] - mu[j] * c[j + 1];
      b[j] = (y[j + 1] - y[j]) - 1 * (c[j + 1] + 2 * c[j]) / 3;
      d[j] = (c[j + 1] - c[j]) / 3;
   }

   for (int i = 0; i < 9; i++) {
      results[4*i+0] = y[i];
      results[4*i+1] = b[i];
      results[4*i+2] = c[i];
      results[4*i+3] = d[i];
   }
}

// Using coeff. calculated above we can approximate y value for x.
// Used to interpolate points set by user
float interpolateSpline(float coeff [40], float x)
{
   x*=9;
   int xx = (int)x;
   float val = x-xx;
   float* cur = &coeff[xx*4];
   return cur[0] + cur[1]*val + cur[2]*val*val + cur[3]*val*val*val;
}

// Alternative interpolation
#if 0
#define min(x,y) (x<y?x:y)

void hermiteCoeff(float y[10], float yd[10], float c[10], float d[10])
{

   for(int i = 0; i < 10; i++)
   {
      yd[i] = 0;

      if (i > 0)
      {
        float xip = 0.1*(i+1);
        float xi = 0.1*i;
        float xim = 0.1*(i-1);

        float hi = xip - xi;
        float him = xi - xim;

        float di = (y[i+1]-y[i])/hi;
        float dim = (y[i]-y[i-1])/him;

        // Steffen
        yd[i] = (copysignf(1, dim) + copysignf(1,di))*min(min(fabs(dim),fabs(di)),fabs(0.5f* (hi*dim + him*di)/(him+hi)));

        // FRitsch-Carlson
        /*  yd[i] = 0;
        if (copysign(1.0,dim) != copysign(1.0,di))
        {
        yd[i] = 3*(him+hi)*1.0/( ((2*hi+him)/dim) + ((hi+2*him)/di));
        }
        */
      }
   }

   for(int i = 0; i < 9; i++)
   {
      float xip = 0.1*(i+1);
      float xi = 0.1*i;
      float xim = 0.1*(i-1);

      float s = (y[i+1]-y[i])/(xip-xi);
      c[i] = (3*s - 2*yd[i] - yd[i+1])/(xip - xi);
      d[i] = (yd[i]+yd[i+1]-2*s)/((xip-xi)*(xip-xi));
   }
}

float heremiteInterpolate(float x, float y[10], float yd[10], float c[10], float d[10])
{
   int idx = (int)(x*10);
   float dxx = x-idx*0.1f;
   return y[idx] + yd[idx]*dxx + c[idx]*dxx*dxx + d[idx]*dxx*dxx*dxx;
}
#endif


typedef struct {
   Vector3 key;
   Vector3 value;
} V3KeyValue;

int v3_hashmap_compare(const void *a, const void *b, void *udata) {
   const float *ua = a;
   const float *ub = b;
   return !(ua[0] == ub[0] && ua[1] == ub[1] && ua[2] == ub[2]);
}

uint64_t v3_hashmap_hash(const void *item, uint64_t seed0, uint64_t seed1) {
   const float *toHash = item;
   float k = (toHash[0]+10000*toHash[1]+1000000*toHash[2]);
   uint32_t h = *((uint32_t*)(&k));
   return ((uint64_t)(h));
}

void recalcNormals(Mesh mesh, int enabled)
{

   struct hashmap *linkedNormals;

   if (enabled)
      linkedNormals = hashmap_new(sizeof(V3KeyValue), 0, 0, 0, v3_hashmap_hash, v3_hashmap_compare, NULL, NULL);

   for(size_t idx = 0; idx < mesh.vertexCount * 3; idx+=9 )
   {
      float *tri = &(mesh.vertices[idx]);

      Vector3 v1 = (Vector3){tri[0], tri[1], tri[2]};
      Vector3 v2 = (Vector3){tri[3], tri[4], tri[5]};
      Vector3 v3 = (Vector3){tri[6], tri[7], tri[8]};

      Vector3 normalV1 = Vector3CrossProduct((Vector3Subtract(v2,v1)), Vector3Subtract(v3,v1));
      Vector3 normalV2 = Vector3CrossProduct((Vector3Subtract(v3,v2)), Vector3Subtract(v1,v2));
      Vector3 normalV3 = Vector3CrossProduct((Vector3Subtract(v1,v3)), Vector3Subtract(v2,v3));

      Vector3 sum = Vector3Add(Vector3Add(normalV1, normalV2), normalV3);

      if (enabled)
      {
         {
            V3KeyValue *item = hashmap_get(linkedNormals, &v1);
            if (item == NULL) hashmap_set(linkedNormals, &(V3KeyValue){ v1, sum });
            else
            {
               item->value.x += sum.x;
               item->value.y += sum.y;
               item->value.z += sum.z;
            }
         }

         {
            V3KeyValue *item = hashmap_get(linkedNormals, &v2);
            if (item == NULL) hashmap_set(linkedNormals, &(V3KeyValue){ v2, sum });
            else
            {
               item->value.x += sum.x;
               item->value.y += sum.y;
               item->value.z += sum.z;
            }
         }

         {
            V3KeyValue *item = hashmap_get(linkedNormals, &v3);
            if (item == NULL) hashmap_set(linkedNormals, &(V3KeyValue){ v3, sum });
            else
            {
               item->value.x += sum.x;
               item->value.y += sum.y;
               item->value.z += sum.z;
            }
         }
      }

      else

      {
         Vector3 norm = Vector3Normalize(sum);

         for(int k = 0; k < 3; ++k)
         {
            mesh.normals[idx + k*3 + 0] = norm.x;
            mesh.normals[idx + k*3 + 1] = norm.y;
            mesh.normals[idx + k*3 + 2] = norm.z;
         }

      }

   }

   if(enabled)
   {
      size_t iter = 0;
      void *item;
      while (hashmap_iter(linkedNormals, &iter, &item)) {
         V3KeyValue *v = item;
         v->value = Vector3Normalize(v->value);
      }

      for(size_t idx = 0; idx < mesh.vertexCount * 3; idx+=3 )
      {
         float *v = &(mesh.vertices[idx]);
         V3KeyValue *item = hashmap_get(linkedNormals, (V3KeyValue*)v);
         memcpy(&(mesh.normals[idx]), &item->value, sizeof(float)*3);
      }

      hashmap_free(linkedNormals);
   }

   UpdateMeshBuffer(mesh, 2, mesh.normals, mesh.vertexCount*3*sizeof(float), 0);
}

float regenerate(bool smooth)
{
   float start = GetTime();
   vase.toUpdate = false;

   float profileCoeff[40];
   naturalSpline(vase.profile, profileCoeff);

   #if 0
   float yy[10], yd[10], c[10], d[10];
   for(int i = 0; i < 10; i++)
   {
      yy[i] = vase.profile[i]/1000.0f;
   }

   hermiteCoeff(yy,yd,c,d);
   #endif



   float layerHeightMm = vase.layerHeight / 1000.0f;
   size_t layersCnt = (size_t)(vase.height/layerHeightMm)+1;

   printf("Layers: %d/%f = %lu\n", vase.height, layerHeightMm, layersCnt);
   printf("Seed: %f\n", 289.0*(vase.noise[0].seed)/1000000000);

   Vector3 *sideMeshVertex;

   #if CALC_NORMALS_WHEN_GENERATE
   Vector3 *sideMeshVertexNormals;
   Vector2  *sideMeshVertexNormalsMap;
   #endif

   printf("Vertex: %u resolution * %lu layers = %lu\n", vase.resolution, layersCnt, layersCnt*vase.resolution);

   /*
      VASE TOP
      --------layersCnt     sideMeshVertex[layersCnt-1]
      --------y2            sideMeshVertex[2]
      --------y1            sideMeshVertex[1]
      --------y0            sideMeshVertex[0]
      xn    x0
   */

   // Ok, we can guess how many triangle we need
   const size_t side_vertex_floats_count =
      (layersCnt-2)         // Number of layers
      * (vase.resolution-1)    // Number of points per layer
      * 2                   // 2 triangle for each point
      * 3                   // 3 coords for each point
      * 3                   // 3 points for each triangle


      + 2                   // Last two layers
      * (vase.resolution-1)    // Number of points per layer
      * 1                   // 1 triangle for each point
      * 3                   // 3 coords for each point
      * 3                   // 3 points for each triangle
      ;

   const size_t base_vertex_floats_count =
      (vase.resolution-1)      // Triangles
      * 3                   // 3 coords for each point
      * 3;                  // 3 points for each triangle

   const size_t total_vertex_floats_count =
      side_vertex_floats_count          // Side
      + 2 * base_vertex_floats_count;   // Top and bottom layer

   printf("side_vertex_floats_count = %lu\n", side_vertex_floats_count);
   printf("base_vertex_floats_count = %lu\n", base_vertex_floats_count);
   printf("total_vertex_floats_count = %lu\n", total_vertex_floats_count);

   Mesh m = {0};
   m.vertexCount = total_vertex_floats_count/3;
   m.triangleCount = total_vertex_floats_count/3/3;

   // Allocate all the space we need for the mesh
   float *meshBuffer = RL_MALLOC(sizeof(float)*total_vertex_floats_count*2);
   m.vertices  = &meshBuffer[0];
   m.normals   = &meshBuffer[total_vertex_floats_count];

   // Allocate all the space we need for our calcs
   #if CALC_NORMALS_WHEN_GENERATE
   Vector3 *tempBuffer = RL_MALLOC(sizeof(Vector3) * vase.resolution * layersCnt * 2 + sizeof(Vector2)*side_vertex_floats_count/3);
   #else
   Vector3 *tempBuffer = RL_MALLOC(sizeof(Vector3) * vase.resolution * layersCnt);
   #endif

   sideMeshVertex              = &tempBuffer[0];

   #if CALC_NORMALS_WHEN_GENERATE
   sideMeshVertexNormals       = &tempBuffer[vase.resolution * layersCnt];
   sideMeshVertexNormalsMap    = (Vector2*)(&tempBuffer[vase.resolution * layersCnt * 2]);
   #endif

printf("TIME #-5: %f\n", GetTime());

   float noiseCoeff[MAX_NOISES][40];

   for(size_t x = 0; x < MAX_NOISES; x++)
   {
      if(vase.noise[x].enabled == false) break;
      naturalSpline(vase.noise[x].alpha, noiseCoeff[x]);
   }


   for(size_t x = 0; x < vase.resolution; ++x)
   {
      for(size_t y = 0; y < layersCnt; y++)
      {
         const size_t idx = x*layersCnt+y;

         float radius = vase.radius;

         radius += vase.profileDepth * interpolateSpline(profileCoeff, y*1.0/layersCnt); // ALSO: heremiteInterpolate(y*1.0/layersCnt*0.9, yy, yd, c, d)

         size_t xx = x%(vase.resolution-1);
         size_t yy = y;

         // Add open simplex noise --->
         for(size_t j = 0; j < MAX_NOISES; j++)
         {
            if (!vase.noise[j].enabled) break;
            if (!vase.noise[j].visible) continue;

            float delta = 1+open_simplex_noise3
            (
               vase.noise[j].ctx,
               vase.noise[j].xScale*0.002*cos(vase.noise[j].twist/360.0f*PI*yy/layersCnt + 2*PI*xx/(vase.resolution-1)),
               vase.noise[j].xScale*0.002*sin(vase.noise[j].twist/360.0f*PI*yy/layersCnt + 2*PI*xx/(vase.resolution-1)),
               vase.noise[j].zScale*0.0001*yy*layerHeightMm
               //289.0*(vase.noise[j].seed)/1000000000
            );

            radius += interpolateSpline(noiseCoeff[j], y*1.0/layersCnt) * delta * vase.noise[j].depth / 2;

         }


         sideMeshVertex[idx] = (Vector3)
         {
            radius*cos(2*PI*xx/(vase.resolution-1)),
            yy*layerHeightMm,
            radius*sin(2*PI*xx/(vase.resolution-1))
         };

         #if CALC_NORMALS_WHEN_GENERATE
         sideMeshVertexNormals[idx] = (Vector3){0,0,0};
         #endif
      }
   }
printf("TIME #-4: %f\n", GetTime());

   // Mesh creation ----->
   size_t globalIdx = 0;
   for(size_t x = 0; x < vase.resolution-1; x++)
   {
      for(size_t y = 0; y < layersCnt; y++)
      {
         if (y > 0)
         {

            float* vertexSlice = &m.vertices[globalIdx*9];

            const Vector3 cur = sideMeshVertex[x*layersCnt + y];
            const Vector3 left = sideMeshVertex[(x+1)*layersCnt + y];
            const Vector3 bottom = sideMeshVertex[(x+1)*layersCnt + y-1];


            vertexSlice[0] = cur.x;
            vertexSlice[1] = cur.y;
            vertexSlice[2] = cur.z;
            vertexSlice[3] = left.x;
            vertexSlice[4] = left.y;
            vertexSlice[5] = left.z;
            vertexSlice[6] = bottom.x;
            vertexSlice[7] = bottom.y;
            vertexSlice[8] = bottom.z;

            #if CALC_NORMALS_WHEN_GENERATE
            Vector2* sideMeshVertexNormalsMapSlice = &sideMeshVertexNormalsMap[globalIdx*3];
            Vector3 normal = Vector3CrossProduct(Vector3Subtract(left, cur), Vector3Subtract(bottom, cur));

            sideMeshVertexNormals[x*layersCnt + y] = Vector3Add(sideMeshVertexNormals[x*layersCnt + y], normal);
            sideMeshVertexNormals[(x+1)*layersCnt + y] = Vector3Add(sideMeshVertexNormals[(x+1)*layersCnt + y], normal);
            sideMeshVertexNormals[(x+1)*layersCnt+(y-1)] = Vector3Add(sideMeshVertexNormals[(x+1)*layersCnt+(y-1)], normal);

            if (x+1 == vase.resolution-1)
            {
               sideMeshVertexNormals[0+y] = Vector3Add(sideMeshVertexNormals[y], normal);
               sideMeshVertexNormals[0+y-1] = Vector3Add(sideMeshVertexNormals[y-1], normal);
            }

            sideMeshVertexNormalsMapSlice[0] = (Vector2){x,y};
            sideMeshVertexNormalsMapSlice[1] = (Vector2){x+1,y};
            sideMeshVertexNormalsMapSlice[2] = (Vector2){x+1,y-1};
            #endif

            globalIdx++;
         }

         if (y < layersCnt-1)
         {

            float* vertexSlice = &m.vertices[globalIdx*9];

            const Vector3 cur = sideMeshVertex[x*layersCnt + y];
            const Vector3 left = sideMeshVertex[(x+1)*layersCnt + y];
            const Vector3 top = sideMeshVertex[x*layersCnt + y+1];

            vertexSlice[0] = cur.x;
            vertexSlice[1] = cur.y;
            vertexSlice[2] = cur.z;
            vertexSlice[3] = top.x;
            vertexSlice[4] = top.y;
            vertexSlice[5] = top.z;
            vertexSlice[6] = left.x;
            vertexSlice[7] = left.y;
            vertexSlice[8] = left.z;

            #if CALC_NORMALS_WHEN_GENERATE
            Vector2* sideMeshVertexNormalsMapSlice = &sideMeshVertexNormalsMap[globalIdx*3];
            Vector3 normal = Vector3CrossProduct(Vector3Subtract(top, cur), Vector3Subtract(left, cur));

            sideMeshVertexNormals[x*layersCnt + y] = Vector3Add(sideMeshVertexNormals[x*layersCnt + y], normal);
            sideMeshVertexNormals[(x+1)*layersCnt + y] = Vector3Add(sideMeshVertexNormals[(x+1)*layersCnt + y], normal);

            if (x+1 == vase.resolution-1) sideMeshVertexNormals[0+y] = Vector3Add(sideMeshVertexNormals[y], normal);

            sideMeshVertexNormals[x*layersCnt+y+1] = Vector3Add(sideMeshVertexNormals[x*layersCnt+y+1], normal);

            sideMeshVertexNormalsMapSlice[0] = (Vector2){x,y};
            sideMeshVertexNormalsMapSlice[1] = (Vector2){x+1,y};
            sideMeshVertexNormalsMapSlice[2] = (Vector2){x,y+1};
            #endif

            globalIdx++;
         }

      }

    }

    #if CALC_NORMALS_WHEN_GENERATE
    for(size_t i = 0; i < side_vertex_floats_count/3; i++)
    {
      // Normals of side mesh
      if (i < side_vertex_floats_count / 3){
         Vector2 nIdx = sideMeshVertexNormalsMap[i];
         Vector3 norm = Vector3Normalize(sideMeshVertexNormals[nIdx.x*layersCnt+nIdx.y]);
         m.normals[0] = norm.x;
         m.normals[1] = norm.y;
         m.normals[2] = norm.z;
      }
    }
    #endif

    // Top and bottom base
printf("TIME #-3: %f\n", GetTime());

   {
      size_t startingIdx = side_vertex_floats_count;
      for(size_t x = 1; x < vase.resolution; ++x)
      {
         Vector3 b = sideMeshVertex[layersCnt*(x-1)];
         Vector3 c = sideMeshVertex[x*layersCnt];

         m.vertices[startingIdx+0] = 0;
         m.vertices[startingIdx+1] = 0;
         m.vertices[startingIdx+2] = 0;

         m.vertices[startingIdx+3] = b.x;
         m.vertices[startingIdx+4] = b.y;
         m.vertices[startingIdx+5] = b.z;


         m.vertices[startingIdx+6] = c.x;
         m.vertices[startingIdx+7] = c.y;
         m.vertices[startingIdx+8] = c.z;

         startingIdx += 9;
      }

      for(size_t x = 1; x < vase.resolution; ++x)
      {
         Vector3 b = sideMeshVertex[x*layersCnt + layersCnt-1];
         Vector3 c = sideMeshVertex[(x-1)*layersCnt + layersCnt-1];

         m.vertices[startingIdx+0] = 0;
         m.vertices[startingIdx+1] = layerHeightMm*(layersCnt-1);
         m.vertices[startingIdx+2] = 0;

         m.vertices[startingIdx+3] = b.x;
         m.vertices[startingIdx+4] = b.y;
         m.vertices[startingIdx+5] = b.z;


         m.vertices[startingIdx+6] = c.x;
         m.vertices[startingIdx+7] = c.y;
         m.vertices[startingIdx+8] = c.z;

         startingIdx += 9;
      }
   }
printf("TIME #-2: %f\n", GetTime());

   #if CALC_NORMALS_WHEN_GENERATE
   // Normals of two bases
   {
      size_t startingIdx = side_vertex_floats_count/3;
      for(size_t i=startingIdx; i < startingIdx+(vase.resolution-1)*3; ++i)
      {
         m.normals[i*3] = 0;
         m.normals[i*3+1] = -1;
         m.normals[i*3+2] = 0;
      }

      startingIdx += (vase.resolution-1)*3;

      for(size_t i=startingIdx; i < startingIdx+(vase.resolution-1)*3; ++i)
      {
         m.normals[i*3] = 0;
         m.normals[i*3+1] = 1;
         m.normals[i*3+2] = 0;
      }
   }
   #endif

printf("TIME #-1: %f\n", GetTime());

   UploadMesh(&m, true);

   if(model.meshCount > 0)
   {
      // NOTE: This will free normals as well!
      RL_FREE(model.meshes[0].vertices);
      model.meshes[0].vertices = 0;
      model.meshes[0].normals = 0;
      UnloadMesh(model.meshes[0]);
      UnloadModel(model);
   }

   printf("TIME #0: %f\n", GetTime());

   model = LoadModelFromMesh(m);
   printf("TIME #1: %f\n", GetTime());

   model.materials[0].shader = shader;
   printf("TIME #2: %f\n", GetTime());
   recalcNormals(model.meshes[0], smooth);
   printf("TIME #3: %f\n", GetTime());

   RL_FREE(tempBuffer);
   printf("TIME #4: %f\n", GetTime());

   return GetTime() - start;
}