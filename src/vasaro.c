#include "raylib.h"
#include "raymath.h"

#include "raygui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CALC_NORMALS_WHEN_GENERATE false

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#include "hashmap.h"
#include "noises/opensimplexnoise.h"
#include "noises/simplexnoise.h"

int screen_w = 640;
int screen_h = 480;

int new_screen_w = 640;
int new_screen_h = 480;

int isPreviewEnabled = 1;
int selectedLayer = 0;

float lastDur = 0;

Vector2 ancModel = (Vector2){ 50, 0 };
Vector2 ancSettings = (Vector2){50, 168 };
Vector2 ancProfile = (Vector2){ 50, 0};

void emscripten_run_script(const char *script);
void emscripten_set_main_loop(void (*func)(), int fps, int simulate_infinite_loop);

Camera  camera;
Shader  shader;
Light   light;
Model   model;

enum {
    LAY_WINDOW_SETTINGS = 0,
    LAY_TEXT_RADIUS,
    LAY_TEXT_HEIGHT,
    LAY_TEXT_POINTS,
    LAY_TEXT_LAYERH,
    LAY_CHECKBOX_FAST,
    LAY_PROFILE_GROUP,
    LAY_BUTTON_PROFILE_THICKNESS,
    LAY_SCROLL_PROFILE_0, LAY_SCROLL_PROFILE_1, LAY_SCROLL_PROFILE_2, LAY_SCROLL_PROFILE_3, LAY_SCROLL_PROFILE_4,
    LAY_SCROLL_PROFILE_5, LAY_SCROLL_PROFILE_6, LAY_SCROLL_PROFILE_7, LAY_SCROLL_PROFILE_8, LAY_SCROLL_PROFILE_9,
    LAY_RENDERING_GROUP,
    LAY_COMBO_COLORS,
    LAY_SMOOTH,
    LAY_AUTOREGEN,
    LAY_COUNT
}
LayoutRects;

Rectangle layoutRects[LAY_COUNT];

Color colors[] =
{
    (Color){204,178,25},
    (Color){240,240,240},
    (Color){164,10,10},
    (Color){25,204,50},
    (Color){25,178,204},
    (Color){204,25,178},
    (Color){50,25,204},
    (Color){30,30,30}
};

void resize(int w, int h)
{
    new_screen_w = w;
    new_screen_h = h;
}

void recalcNormals(Mesh mesh, int enabled);
void UpdateDrawFrame();

#define MAX_NOISES 10

typedef struct {
    bool        enabled;
    int64_t     seed;
    Vector3     direction;
    float       height;
    float       alpha[10];

    struct osn_context *ctx;
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

#define VASE_RADIUS_MIN 10
#define VASE_RADIUS_MAX 80

#define VASE_HEIGHT_MIN 1
#define VASE_HEIGHT_MAX 200

#define VASE_RESOLUTION_MIN 3
#define VASE_RESOLUTION_MAX 500

#define VASE_LAYER_HEIGHT_MIN 50
#define VASE_LAYER_HEIGHT_MAX 200*1000

Vase v;

void initGui();
void renderGui();
bool isEditing();

void BtnNew();
void BtnExport();

bool smoothModel = false;
bool autoRegenerate = true;

bool isWindowVisible[2] = {false, false};

Font font;

void initVase()
{
    v.toUpdate = true;
    v.height = 100;
    v.radius = 20;
    v.resolution = 300;
    v.layerHeight = 200;
    v.color = 0;

    v.profileDepth = 10;
    for(size_t i = 0; i< 10; i++)
        v.profile[i] = GetRandomValue(0,1000);

    for(size_t i = 0; i<MAX_NOISES; ++i)
    {
        v.noise[i].enabled = false;

        for(size_t k = 0; k < 10; ++k)
            v.noise[i].alpha[k] = 1.0;

        v.noise[i].direction = (Vector3){1,0,1};
        v.noise[i].height = 20;
        v.noise[i].seed = GetRandomValue(-1000000000, 1000000000);
        v.noise[i].ctx = NULL;
    }
}

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

void regenerate()
{
    float start = GetTime();
    v.noise[0].enabled = true;
    v.noise[0].height = 10;
    v.toUpdate = false;

    float profileCoeff[40];
    naturalSpline(v.profile, profileCoeff);
    //open_simplex_noise(v.noise[0].seed, &v.noise[0].ctx);

    float layerHeightMm = v.layerHeight / 1000.0f;
    size_t layersCnt = (size_t)(v.height/layerHeightMm)+1;

    printf("Layers: %d/%f = %lu\n", v.height, layerHeightMm, layersCnt);
    printf("Seed: %f\n", 289.0*(v.noise[0].seed)/1000000000);

    Vector3 *sideMeshVertex;

    #if CALC_NORMALS_WHEN_GENERATE
    Vector3 *sideMeshVertexNormals;
    Vector2  *sideMeshVertexNormalsMap;
    #endif

    printf("Vertex: %u resolution * %lu layers = %lu\n", v.resolution, layersCnt, layersCnt*v.resolution);

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
      * (v.resolution-1)    // Number of points per layer
      * 2                   // 2 triangle for each point
      * 3                   // 3 coords for each point
      * 3                   // 3 points for each triangle


      + 2                   // Last two layers
      * (v.resolution-1)    // Number of points per layer
      * 1                   // 1 triangle for each point
      * 3                   // 3 coords for each point
      * 3                   // 3 points for each triangle
      ;

    const size_t base_vertex_floats_count =
      (v.resolution-1)      // Triangles
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
    Vector3 *tempBuffer = RL_MALLOC(sizeof(Vector3) * v.resolution * layersCnt * 2 + sizeof(Vector2)*side_vertex_floats_count/3);
    #else
    Vector3 *tempBuffer = RL_MALLOC(sizeof(Vector3) * v.resolution * layersCnt);
    #endif

    sideMeshVertex              = &tempBuffer[0];

    #if CALC_NORMALS_WHEN_GENERATE
    sideMeshVertexNormals       = &tempBuffer[v.resolution * layersCnt];
    sideMeshVertexNormalsMap    = (Vector2*)(&tempBuffer[v.resolution * layersCnt * 2]);
    #endif

printf("TIME #-5: %f\n", GetTime());

    for(size_t i = 0; i< 10; i++)
    {
        printf("V: %f\n", interpolateSpline(profileCoeff, i*1.0/10));
    }
    for(size_t x = 0; x < v.resolution; ++x)
    {
        for(size_t y = 0; y < layersCnt; y++)
        {
            const size_t idx = x*layersCnt+y;

            float radius = v.radius;

            radius += v.profileDepth * interpolateSpline(profileCoeff, y*1.0/layersCnt);

            size_t xx = x%(v.resolution-1);
            size_t yy = y;

            // Add open simplex noise --->
            for(size_t j = 0; j < MAX_NOISES; j++)
            {
                if (!v.noise[j].enabled) continue;

                float delta = 1+snoise4
                (
                    0.02*v.radius*cos(2*PI*xx/(v.resolution-1)),
                    0.02*v.radius*sin(2*PI*xx/(v.resolution-1)),
                    0.02*yy*layerHeightMm,
                    289.0*(v.noise[j].seed)/1000000000
                );


                radius += delta * v.noise[j].height;
            }

            sideMeshVertex[idx] = (Vector3)
            {
                radius*cos(2*PI*xx/(v.resolution-1)),
                yy*layerHeightMm,
                radius*sin(2*PI*xx/(v.resolution-1))
            };

            #if CALC_NORMALS_WHEN_GENERATE
            sideMeshVertexNormals[idx] = (Vector3){0,0,0};
            #endif
        }
    }

printf("TIME #-4: %f\n", GetTime());

    // Mesh creation ----->
    size_t globalIdx = 0;
    for(size_t x = 0; x < v.resolution-1; x++)
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

                if (x+1 == v.resolution-1)
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

                if (x+1 == v.resolution-1) sideMeshVertexNormals[0+y] = Vector3Add(sideMeshVertexNormals[y], normal);

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
        for(size_t x = 1; x < v.resolution; ++x)
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

        for(size_t x = 1; x < v.resolution; ++x)
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
        for(size_t i=startingIdx; i < startingIdx+(v.resolution-1)*3; ++i)
        {
           m.normals[i*3] = 0;
           m.normals[i*3+1] = -1;
           m.normals[i*3+2] = 0;
        }

        startingIdx += (v.resolution-1)*3;

        for(size_t i=startingIdx; i < startingIdx+(v.resolution-1)*3; ++i)
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
    recalcNormals(model.meshes[0], smoothModel);
    printf("TIME #3: %f\n", GetTime());

    RL_FREE(tempBuffer);
    printf("TIME #4: %f\n", GetTime());

    lastDur = GetTime() - start;

}

Texture icons[4][2];
Texture refreshIcon;

const char* iconsDescr[4] = { "NEW VASE", "DOWNLOAD THIS VASE", "VASE SETTINGS", "NOISE SETTINGS"};

int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screen_w, screen_h, "Vasaro");
    resize(screen_w, screen_h);

    initVase();
    initGui();

    font =  LoadFontEx("resources/Zector.ttf", 20, NULL, 0);  // Load font from file with extended parameters, use NULL for fontChars and 0 for glyphCount to load the default character set


    icons[0][1] = LoadTexture("resources/icons/new.empty.png");
    icons[0][0] = LoadTexture("resources/icons/new.png");

    icons[1][0] = LoadTexture("resources/icons/download.png");
    icons[1][1] = LoadTexture("resources/icons/download.empty.png");

    icons[2][0] = LoadTexture("resources/icons/vase.png");
    icons[2][1] = LoadTexture("resources/icons/vase.empty.png");

    icons[3][0] = LoadTexture("resources/icons/layers.png");
    icons[3][1] = LoadTexture("resources/icons/layers.empty.png");

    refreshIcon = LoadTexture("resources/icons/refresh.png");

    // Init resources
    // Camera is fixed

	camera = (Camera)
	{
		(Vector3){0,150,10}, 		// It is placed away from center
		(Vector3){0, 50.0f, 0.0f},  // Watching the center of the object
		(Vector3){0.0f, 1.0f, 0.0f}	// Up direction
		,45.0f, CAMERA_PERSPECTIVE
    };

	// Light effect shader
	shader = LoadShader("resources/lightning.vs","resources/lightning.fs");

    // Shader setup
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    float shaderValue[] = { 0.5f, 0.5f, 0.5f, 1.0f };

	int ambientLoc = GetShaderLocation(shader, "ambient");
    SetShaderValue(shader, ambientLoc, shaderValue, SHADER_UNIFORM_VEC4);

	// Just one light.
	light = CreateLight(LIGHT_POINT, (Vector3){6, 10, 10}, (Vector3){0, 0, 0}, (Color){255,255,255,255}, shader);

    regenerate();


    GuiLoadStyle("resources/style.rgs");

    emscripten_run_script("update_canvas_size();");
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);

    CloseWindow();        // Close window and OpenGL context

    return 0;
}

void UpdateDrawFrame()
{

    // First check: resize window if needed
    if (IsWindowResized() || new_screen_w != screen_w ||new_screen_h != screen_h)
    {
        screen_w = new_screen_w;
        screen_h = new_screen_h;
        SetWindowSize(screen_w, screen_h);
        initGui();
    }

    static float modelRotationX = 45;
    static float cameraHeight = 150;
    static bool isOverGUI = false;

    static bool wasButtonDown = false;

    static int draggedWindow = -1;
    int hittedWindow = -1;

    if (v.toUpdate && autoRegenerate && IsMouseButtonUp(0))
        regenerate();


    Vector2 mouse = GetMousePosition();

    hittedWindow = -1;

    if (IsMouseButtonUp(0))
        draggedWindow = -1;

    if (IsMouseButtonDown(0) && draggedWindow >= 0)
    {
        if (draggedWindow == LAY_WINDOW_SETTINGS)
        {
            ancSettings.x += GetMouseDelta().x;
            ancSettings.y += GetMouseDelta().y;

            if (ancSettings.x < 50) ancSettings.x = 50;
            initGui();
        }
    }

    for(int r = 0; r < 2; r++)
    {
        if (isWindowVisible[r] && CheckCollisionPointRec(mouse, layoutRects[r]))
        {
            hittedWindow = r;

            Rectangle header = layoutRects[r];
            header.height = 24;

            if (IsMouseButtonDown(0) && !wasButtonDown && CheckCollisionPointRec(mouse, header))
            {
                draggedWindow = r;
            }
            break;
        }
    }



    isOverGUI = hittedWindow >= 0 || draggedWindow >= 0;

    if (!autoRegenerate && v.toUpdate &&
        !isOverGUI && !wasButtonDown && IsMouseButtonDown(0)
        && mouse.x > screen_w - 32- 20 && mouse.x < screen_w - 20
        && mouse.y > 20 && mouse.y < 20 + 32
        )
        regenerate();

    {
        // Move model
        {
            static float speedX = 0;
            if (IsMouseButtonDown(0) && !isOverGUI)
            {
                {
                    Vector2 delta = GetMouseDelta();
                    if (fabs(delta.x) < 5) speedX = 0;
                    else speedX = delta.x;

                    if (speedX > 40) speedX = 40;
                    else if (speedX < -40) speedX = -40;

                    modelRotationX += delta.x*1.0f/3.5;
                    cameraHeight += delta.y*1.0f;

                    if (cameraHeight > 200) cameraHeight = 200;
                    if (cameraHeight < -200) cameraHeight = -200;
                }

            }
            else
            {
                modelRotationX += speedX*1.0f/4;

                float delta = 20*GetFrameTime();
                if (speedX < 0) delta *= -1;

                if (fabs(delta) > fabs(speedX)) speedX = 0;
                else speedX -= delta;

                if (fabs(speedX) < 0.1)
                    speedX = 0;
            }
        }
    }

    //recalcNormals(model.meshes[0], 1);

    // Update view
    {
        //camera.position.y -= GetFrameTime();
        camera.position = (Vector3){ 160*cos(DEG2RAD*modelRotationX), cameraHeight,sin(DEG2RAD*modelRotationX)*160};
        light.position = camera.position;
        light.position.x += 5;
        light.position.y += 2;



        UpdateCamera(&camera);

        // Update shader
        float cameraPos[] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(shader, shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        // Update light
        UpdateLightValues(shader, light);
    }

    BeginDrawing();

        ClearBackground((Color){25, 46,61, 255});

        {
            BeginMode3D(camera);

            DrawGrid(10, 19);

            DrawModelEx(model, (Vector3){0,0,0}, (Vector3){0,1,0}, 0, (Vector3){1,1,1}, colors[v.color]);
            //DrawModel(model, (Vector3){0,0,0}, 1, colors[1]);
            EndMode3D();
        }

        if (v.toUpdate && !autoRegenerate)
            DrawTexture(refreshIcon, screen_w - 32 - 20, 20, (Color){200,200,200,100});

        // Draw UI



        Color c = {100,100,100,200};
        Color c1 = {180,180,180,200};

        DrawRectangle(0,0,48,screen_h, (Color){15,36,51,230});

        for(int i = 0; i < 4; ++i)
        {
            if (CheckCollisionPointRec(mouse, (Rectangle){8,8*(i+1)+40*i,32,32}))
            {
                DrawTexture(icons[i][0],8,8*(i+1)+40*i,c1);
                DrawTextPro(font, iconsDescr[i], (Vector2) {33, 210}, (Vector2) {0, 0}, 90, 20, 3, (Color){180,180,180,200});

                if(IsMouseButtonPressed(0))
                {
                    switch(i)
                    {
                        case 0:
                            BtnNew();
                            break;

                        case 1:
                            BtnExport();
                            break;

                        case 2:
                            isWindowVisible[LAY_WINDOW_SETTINGS] = true;
                            ancSettings.x = 50;
                            ancSettings.y = 8*(i+1)+40*i;
                            initGui();
                            break;

                        case 3:
                            break;
                    }
                }


            }
            else
                DrawTexture(icons[i][1],8,8*(i+1)+40*i,c);
        }

        renderGui();

        static bool lastEditingState = false;

        if (lastEditingState != isEditing())
        {
            lastEditingState = isEditing();
            if (lastEditingState == false) v.toUpdate = true;
        }

        char stats[200];
        snprintf(stats, 200, "%lu TRIANGLES GENERATED IN %0.3fs", model.meshes[0].triangleCount, lastDur);
        DrawText(stats, 60, screen_h - 20, 10, (Color){230,230,230,200});
    EndDrawing();

    wasButtonDown = IsMouseButtonDown(0);

    //----------------------------------------------------------------------------------
}

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

void BtnExport()
{
    if (v.toUpdate)
        regenerate();

    uint32_t        triCount    = model.meshes[0].vertexCount / 3;
    unsigned int    FILE_SIZE   = 80 + sizeof(uint32_t) + (sizeof(float)*3 + sizeof(float)*9 + sizeof(uint16_t)) * triCount;

    // Clear file content
    char *content = calloc(1, FILE_SIZE);

    // Add a signature
    strcpy(content, "Made with vasaro-web. https://andreafontana.it/vasaro-web/");

    // Number of triangles
    *((uint32_t*)(&content[80])) = triCount;

    // Vertices of current mesh
    float *vertices = model.meshes[0].vertices;
    for(size_t i = 0; i < triCount; i++)
    {
        // A couple of shortcuts
        float *base = (float*)&content[80 + sizeof(uint32_t) + i*(sizeof(float)*3 + sizeof(float)*9 + sizeof(uint16_t))];
        float *vbase = &vertices[i*9];

        // Normals are ignored by design
        base[0] = 0;
        base[1] = 0;
        base[2] = 0;

        // Switching XYZ axes to match stl editors
        base[3] = vbase[0];
        base[4] = vbase[2];
        base[5] = vbase[1];
        base[6] = vbase[6];
        base[7] = vbase[8];
        base[8] = vbase[7];
        base[9] = vbase[3];
        base[10] = vbase[5];
        base[11] = vbase[4];
    }

    SaveFileData("model.stl", content, FILE_SIZE);
    emscripten_run_script("saveFile();");

    free(content);
}



void initGui(void)
{
    layoutRects[LAY_WINDOW_SETTINGS] = (Rectangle){ ancSettings.x + 8, ancSettings.y + 0, 360, 435 };
    layoutRects[LAY_TEXT_RADIUS] = (Rectangle){ ancSettings.x + 116, ancSettings.y + 40, 64, 24 };
    layoutRects[LAY_TEXT_HEIGHT] = (Rectangle){ ancSettings.x + 116, ancSettings.y + 72, 64, 24 };
    layoutRects[LAY_TEXT_LAYERH] = (Rectangle){ ancSettings.x + 116, ancSettings.y + 104, 64, 24 };
    layoutRects[LAY_TEXT_POINTS] = (Rectangle){ ancSettings.x + 116, ancSettings.y + 136, 64, 24 };
    layoutRects[LAY_COMBO_COLORS] = (Rectangle){ ancSettings.x + 16,  ancSettings.y + 212, 120, 24 };


    layoutRects[LAY_PROFILE_GROUP] = (Rectangle) { ancSettings.x + 194, ancSettings.y + 8 + 24 + 8, 160, 380};

    Vector2 ancProfile = { layoutRects[LAY_PROFILE_GROUP].x + 20, layoutRects[LAY_PROFILE_GROUP].y + 10 };

    layoutRects[LAY_BUTTON_PROFILE_THICKNESS] = (Rectangle) { ancProfile.x + 40, ancProfile.y + 10, 80, 24};

    for(size_t i = 0; i < 10; ++i)
        layoutRects[LAY_SCROLL_PROFILE_9 - i] = (Rectangle){ ancProfile.x, ancProfile.y + 32*i+40, 120, 24 };


    layoutRects[LAY_RENDERING_GROUP] = (Rectangle) { ancSettings.x + 20, ancSettings.y + 280, 160, 140};
    Vector2 ancRendering = { layoutRects[LAY_RENDERING_GROUP].x + 10, layoutRects[LAY_RENDERING_GROUP].y + 10 };
    layoutRects[LAY_COMBO_COLORS] = (Rectangle){ ancRendering.x + 10,  ancRendering.y + 10, 120, 24 };
    layoutRects[LAY_SMOOTH] = (Rectangle){ ancRendering.x + 10,  ancRendering.y + 10 + 40, 120, 24 };
    layoutRects[LAY_AUTOREGEN] = (Rectangle){ ancRendering.x + 10,  ancRendering.y + 10 + 40*2, 120, 24 };





}

void BtnNew()
{
    initVase();
    isWindowVisible[0] = false;
    isWindowVisible[1] = false;
}

bool vaseHeightEditMode = false;
bool vaseLayerHeightEditMode = false;
bool vasePointsEditMode = false;
bool vaseRadiusEditMode = false;
bool showProfileWindow = false;
bool vaseProfileDepthEditMode = false;

bool isEditing()
{
   return vaseProfileDepthEditMode || vaseHeightEditMode || vaseLayerHeightEditMode || vasePointsEditMode || vaseRadiusEditMode;
}

void renderGui()
{
    //bool isProfileWindowOpen = showProfileWindow;

    static bool profileChanged = false;

    //if (isProfileWindowOpen) GuiDisable();

    if (isWindowVisible[LAY_WINDOW_SETTINGS])
    {
        isWindowVisible[LAY_WINDOW_SETTINGS] = !GuiWindowBox(layoutRects[LAY_WINDOW_SETTINGS], "VASE SETTINGS");
        if (GuiValueBox(layoutRects[LAY_TEXT_RADIUS], "RADIUS (MM) ", &v.radius, VASE_RADIUS_MIN, VASE_RADIUS_MAX, vaseRadiusEditMode)) vaseRadiusEditMode = !vaseRadiusEditMode;
        if (GuiValueBox(layoutRects[LAY_TEXT_HEIGHT], "HEIGHT (MM) ", &v.height, VASE_HEIGHT_MIN, VASE_HEIGHT_MAX, vaseHeightEditMode)) vaseHeightEditMode = !vaseHeightEditMode;
        if (GuiValueBox(layoutRects[LAY_TEXT_POINTS], "BASE VERTICES ", &v.resolution, VASE_RESOLUTION_MIN, VASE_RESOLUTION_MAX, vasePointsEditMode)) vasePointsEditMode = !vasePointsEditMode;

        int maxLayers = 1000*v.height;
        if (maxLayers < VASE_LAYER_HEIGHT_MIN) maxLayers = v.layerHeight;
        if (GuiValueBox(layoutRects[LAY_TEXT_LAYERH], "LAYER (MICRON) ", &v.layerHeight, VASE_LAYER_HEIGHT_MIN, maxLayers, vaseLayerHeightEditMode)) vaseLayerHeightEditMode = !vaseLayerHeightEditMode;




        GuiGroupBox(layoutRects[LAY_PROFILE_GROUP], "PROFILE");

        if (GuiValueBox(layoutRects[LAY_BUTTON_PROFILE_THICKNESS], "DEPTH ", &v.profileDepth, 1, v.radius, vaseProfileDepthEditMode)) vaseProfileDepthEditMode = !vaseProfileDepthEditMode;

        bool isStillOver = false;
        for(size_t i = 0; i < 10; i++)
        {
            int32_t old = v.profile[i];
            v.profile[i] = GuiSlider(layoutRects[LAY_SCROLL_PROFILE_0 + i], "", "", v.profile[i], 0, 1000);
            if (old != v.profile[i]) profileChanged = true;

            if(!isStillOver && CheckCollisionPointRec(GetMousePosition(), layoutRects[LAY_SCROLL_PROFILE_0 +i]) && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            {
                isStillOver = true;

            }

        }

        GuiGroupBox(layoutRects[LAY_RENDERING_GROUP], "RENDERING");
        v.color = GuiComboBox(layoutRects[LAY_COMBO_COLORS], "YELLOW;WHITE;RED;GREEN;CYAN;MAGENTA;LILAC;GRAY", v.color);

        bool wasSmooth = smoothModel;
        smoothModel = GuiToggle(layoutRects[LAY_SMOOTH], "SMOOTH MODEL", smoothModel);
        if (wasSmooth != smoothModel) v.toUpdate = true;

        autoRegenerate = GuiToggle(layoutRects[LAY_AUTOREGEN], "AUTO REGENERATE", autoRegenerate);


        if (!isStillOver && profileChanged)
        {
            v.toUpdate = true;
            profileChanged = false;
        }
    }

    if (!isEditing())
    {
        if (v.radius < VASE_RADIUS_MIN) v.radius = VASE_RADIUS_MIN;
        if (v.radius > VASE_RADIUS_MAX) v.radius = VASE_RADIUS_MAX;

        if (v.layerHeight < VASE_LAYER_HEIGHT_MIN) v.layerHeight = VASE_LAYER_HEIGHT_MIN;
        if (v.layerHeight > VASE_LAYER_HEIGHT_MAX) v.layerHeight = VASE_LAYER_HEIGHT_MAX;

        if (v.height < VASE_HEIGHT_MIN) v.height = VASE_HEIGHT_MIN;
        if (v.height > VASE_HEIGHT_MAX) v.height = VASE_HEIGHT_MAX;

        if (v.resolution < VASE_RESOLUTION_MIN) v.resolution = VASE_RESOLUTION_MIN;
        if (v.resolution > VASE_RESOLUTION_MAX) v.resolution = VASE_RESOLUTION_MAX;
    }

/*
    if (isProfileWindowOpen)
    {
        GuiEnable();
        if (GuiWindowBox(layoutRects[LAY_WINDOW_PROFILE], "PROFILE")) showProfileWindow = false;



        GuiDisable();
    }

    v.color = GuiComboBox(layoutRects[LAY_COMBO_COLORS], "YELLOW;WHITE;RED;GREEN;CYAN;MAGENTA;LILAC;GRAY", v.color);
    if (GuiValueBox(layoutRects[LAY_TEXT_RADIUS], "RADIUS: ", &v.radius, VASE_RADIUS_MIN, VASE_RADIUS_MAX, vaseRadiusEditMode)) vaseRadiusEditMode = !vaseRadiusEditMode;




    if (isProfileWindowOpen)
        GuiEnable();

        */

}