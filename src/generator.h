#ifndef GENERATOR_H
#define GENERATOR_H

#include <stdint.h>
#include <stddef.h>

#include "raylib.h"
#include "raymath.h"

#define MAX_NOISES 10

typedef struct {
    size_t      noiseId;
    bool        enabled;
    bool        visible;
    int64_t     seed;
    Vector3     direction;
    float       height;
    float       alpha[10];
} Noise;

typedef struct {
    int32_t    height;         // 100
    int32_t    radius;       // 60
    int32_t    resolution;     // 3032
    int32_t    layerHeight;    // 0.2

    int32_t     profile[10];
    int32_t     profileDepth;

    bool        toUpdate;
    Noise       noise[MAX_NOISES];

    size_t      color;
} Vase;

float regenerate(bool smooth);

#endif