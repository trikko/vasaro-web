#ifndef GENERATOR_H
#define GENERATOR_H

#include <stdint.h>
#include <stddef.h>

#include "raylib.h"
#include "raymath.h"
#include "noises/opensimplexnoise.h"

#define MAX_NOISES 10

typedef struct {
   size_t      noiseId;
   bool        enabled;
   bool        visible;
   int64_t     seed;

   int32_t     xScale;
   int32_t     yScale;
   int32_t     zScale;
   int32_t     twist;
   int32_t     depth;
   int32_t     alpha[10];

   struct osn_context *ctx;
} Noise;

typedef struct {
   int32_t    height;         // 100
   int32_t    radius;       // 60
   int32_t    resolution;     // 3032
   int32_t    layerHeight;    // 0.2

   int32_t     profile[10];
   int32_t     profileDepth;

   float       maxRadius;
   bool        toUpdate;
   bool        isEditing;

   Noise       noise[MAX_NOISES];

   size_t      color;
} Vase;

float regenerate(bool smooth);

#endif