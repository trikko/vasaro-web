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
int screen_h = 640;

int new_screen_w = 640;
int new_screen_h = 640;

int isPreviewEnabled = 1;
int  selectedLayer = 0;

void emscripten_run_script(const char *script);
void emscripten_set_main_loop(void (*func)(), int fps, int simulate_infinite_loop);

Camera  camera;
Shader  shader;
Light   light;
Model   model;

enum {
    LAY_BUTTON_EXPORT = 0,
    LAY_BUTTON_NEW,
    LAY_WINDOW_MODEL,
    LAY_WINDOW_SETTINGS,
    LAY_COMBO_COLORS,
    LAY_TEXT_RADIUS,
    LAY_TEXT_HEIGHT,
    LAY_TEXT_POINTS,
    LAY_TEXT_LAYERH,
    LAY_CHECKBOX_FAST,
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

    bool        toUpdate;
    Noise       noise[MAX_NOISES];

    size_t      color;
} Vase;

#define VASE_RADIUS_MIN 20
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

bool smoothModel = false;

void initVase()
{
    v.toUpdate = true;
    v.height = 100;
    v.radius = 50;
    v.resolution = 300;
    v.layerHeight = 200;
    v.color = 0;

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

void regenerate()
{
    v.noise[0].enabled = true;
    v.noise[0].height = 10;
    v.toUpdate = false;

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

    for(size_t x = 0; x < v.resolution; ++x)
    {
        for(size_t y = 0; y < layersCnt; y++)
        {
            const size_t idx = x*layersCnt+y;

            float radius = v.radius;

            size_t xx = x%(v.resolution-1);
            size_t yy = y;

            // Add open simplex noise --->
            for(size_t j = 0; j < MAX_NOISES; j++)
            {
                if (!v.noise[j].enabled) continue;

                float delta = snoise4
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

}

int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screen_w, screen_h, "Vasaro");
    resize(screen_w, screen_h);

    initVase();
    initGui();

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
    }

    static float modelRotationX = 45;
    static float cameraHeight = 150;
    static bool isOverGUI = false;

    static bool         showVaseSettings = false;
    static Rectangle    vaseSettingsRect;

    if (v.toUpdate && IsMouseButtonUp(0))
    {
        regenerate();
    }


    Vector2 mouse = GetMousePosition();

    isOverGUI = false;
    for(int r = 0; r < 9; r++)
    {

        if (CheckCollisionPointRec(mouse, layoutRects[r]))
        {
            isOverGUI = true;
            break;
        }
    }

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

                    modelRotationX += delta.x*1.0f/4;
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

                if (delta > speedX) speedX = 0;
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
            DrawModelEx(model, (Vector3){0,0,0}, (Vector3){0,1,0}, modelRotationX, (Vector3){1,1,1}, colors[v.color]);
            //DrawModel(model, (Vector3){0,0,0}, 1, colors[1]);
            EndMode3D();
        }

        // Draw UI

        renderGui();

        static bool lastEditingState = false;

        if (lastEditingState != isEditing())
        {
            lastEditingState = isEditing();
            if (lastEditingState == false) regenerate();
        }

        /*
        // --- ACTIONS
        const int GUI_SPACING = 15;
        GuiWindowBox((Rectangle){GUI_SPACING, GUI_SPACING, 180, GUI_SPACING + 20 + GUI_SPACING*3 + 30*3}, "#198# ACTIONS");

        isPreviewEnabled = GuiToggle((Rectangle){GUI_SPACING*2, GUI_SPACING + 20 + GUI_SPACING, 180-15*2, 30}, "#12# LIVE PREVIEW", isPreviewEnabled);

        // Export Model
        if (GuiButton((Rectangle){GUI_SPACING*2, GUI_SPACING + 20 + GUI_SPACING*2 + 30, 180-15*2, 30}, "#2# EXPORT MODEL"))
        {

        }

        // About Vasaro
        if (GuiButton((Rectangle){GUI_SPACING*2, GUI_SPACING + 20 + GUI_SPACING*3 + 30*2, 180-15*2, 30}, "#193# ABOUT VASARO"))
        {

        }

        // --- VASE SETTINGS
        const int SETTINGS_TOP = GUI_SPACING + 20 + GUI_SPACING*5 + 30*3;
        GuiWindowBox((Rectangle){GUI_SPACING, SETTINGS_TOP, 180, GUI_SPACING + 20 + GUI_SPACING*2 + 30*2}, "#140# VASE PARAMS");

        // Dimensions
        if (GuiButton((Rectangle){GUI_SPACING*2, SETTINGS_TOP + 20 + GUI_SPACING, 180-15*2, 30}, "#69# SETTINGS ..."))
        {
            showVaseSettings = true;
            vaseSettingsRect = (Rectangle){screen_w/2 - 150, screen_h/2 - 100, 300, 200};
        }

        if (showVaseSettings)
        {
            if (GuiWindowBox(vaseSettingsRect, "Vase settings"))
            {
                showVaseSettings = false;
            }

            float newDiameter = GuiSlider((Rectangle){vaseSettingsRect.x+GUI_SPACING+100, vaseSettingsRect.y+GUI_SPACING+20, 100,30}, "DIAMETER: ", "", v.diameter, 20,120);

            if (newDiameter != v.diameter)
            {
                v.toUpdate = true;
                v.diameter = newDiameter;
            }
        }

        // Dimensions
        if (GuiButton((Rectangle){GUI_SPACING*2, SETTINGS_TOP + 20 + GUI_SPACING*2 + 30, 180-15*2, 30}, "#22# EDIT PROFILE ..."))
        {

        }

        // --- NOISE LAYERS
        const int NOISE_TOP = SETTINGS_TOP + 20 + GUI_SPACING*4 + 30*2;
        GuiWindowBox((Rectangle){GUI_SPACING, NOISE_TOP, 180, screen_h - NOISE_TOP - GUI_SPACING}, "#95# NOISE LAYERS");

        // ADD
        if (GuiButton((Rectangle){GUI_SPACING*2, NOISE_TOP + 20 + GUI_SPACING, 30, 30}, "#8#"))
        {

        }

        if (GuiButton((Rectangle){GUI_SPACING*2 + 30 + 10, NOISE_TOP + 20 + GUI_SPACING, 30, 30}, "#9#"))
        {
        }

        if (GuiButton((Rectangle){GUI_SPACING*2 + 30*2 + 10*2, NOISE_TOP + 20 + GUI_SPACING, 30, 30}, "#44#"))
        {
        }

        if (GuiButton((Rectangle){GUI_SPACING*2 + 30*3 + 10*3, NOISE_TOP + 20 + GUI_SPACING, 30, 30}, "#45#"))
        {
        }

        const char* lv = "LAYER #1;LAYER #2";
        int a;
        selectedLayer = GuiListView((Rectangle){GUI_SPACING*2, NOISE_TOP + 20 + GUI_SPACING*2 + 30, 180-30, screen_h - GUI_SPACING - NOISE_TOP - 20 - GUI_SPACING - 30*2}, lv, &a, selectedLayer);


        if (IsMouseButtonPressed(0))
            validModelClick = GetMousePosition().x > 180;
        // VISIBLE

        // INVISIBLE
        //DrawText(s, 190, 200, 20, (Color){200,200,100,100});
*/
    EndDrawing();

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
    Vector2 anchor01 = (Vector2){ 0, 0 };
    Vector2 anchor02 = (Vector2){ 0, 168 };

    layoutRects[LAY_BUTTON_EXPORT] = (Rectangle){ anchor01.x + 16, anchor01.y+ 88, 120, 24 };
    layoutRects[LAY_BUTTON_NEW] = (Rectangle){ anchor01.x + 16, anchor01.y + 48, 120, 24 };
    layoutRects[LAY_CHECKBOX_FAST] = (Rectangle){ anchor01.x + 16, anchor01.y + 128, 16, 16 };

    layoutRects[LAY_WINDOW_MODEL] = (Rectangle){ anchor01.x + 8, anchor01.y + 8, 136, 144+8 };
    layoutRects[LAY_WINDOW_SETTINGS] = (Rectangle){ anchor02.x + 8, anchor02.y + 0, 135, 216 };
    layoutRects[LAY_TEXT_RADIUS] = (Rectangle){ anchor02.x + 76, anchor02.y + 40, 60, 24 };
    layoutRects[LAY_TEXT_HEIGHT] = (Rectangle){ anchor02.x + 76, anchor02.y + 72, 60, 24 };
    layoutRects[LAY_TEXT_POINTS] = (Rectangle){ anchor02.x + 76, anchor02.y + 104, 60, 24 };
    layoutRects[LAY_TEXT_LAYERH] = (Rectangle){ anchor02.x + 76, anchor02.y + 136, 60, 24 };
    layoutRects[LAY_COMBO_COLORS] = (Rectangle){ anchor02.x + 16,  anchor02.y + 180, 120, 24 };


}

void BtnNew()
{
    initVase();
}

bool vaseHeightEditMode = false;
bool vaseLayerHeightEditMode = false;
bool vasePointsEditMode = false;
bool vaseRadiusEditMode = false;


bool isEditing()
{
   return vaseHeightEditMode || vaseLayerHeightEditMode || vasePointsEditMode || vaseRadiusEditMode;
}

void renderGui()
{

    GuiWindowBox(layoutRects[LAY_WINDOW_MODEL], "MODEL");
    if (GuiButton(layoutRects[LAY_BUTTON_EXPORT], "EXPORT AS STL")) BtnExport();
    if (GuiButton(layoutRects[LAY_BUTTON_NEW], "NEW VASE")) BtnNew();
    if (smoothModel != GuiCheckBox(layoutRects[LAY_CHECKBOX_FAST], " SMOOTH MODEL", smoothModel))
    {
        smoothModel = !smoothModel;
        v.toUpdate = true;
    }


    GuiWindowBox(layoutRects[LAY_WINDOW_SETTINGS], "SETTINGS");
    if (GuiValueBox(layoutRects[LAY_TEXT_HEIGHT], "HEIGHT: ", &v.height, VASE_HEIGHT_MIN, VASE_HEIGHT_MAX, vaseHeightEditMode)) vaseHeightEditMode = !vaseHeightEditMode;
    if (GuiValueBox(layoutRects[LAY_TEXT_POINTS], "POINTS: ", &v.resolution, VASE_RESOLUTION_MIN, VASE_RESOLUTION_MAX, vasePointsEditMode)) vasePointsEditMode = !vasePointsEditMode;

    int maxLayers = 1000*v.height;
    if (maxLayers < VASE_LAYER_HEIGHT_MIN) maxLayers = v.layerHeight;

    if (GuiValueBox(layoutRects[LAY_TEXT_LAYERH], "LAYER H.: ", &v.layerHeight, VASE_LAYER_HEIGHT_MIN, maxLayers, vaseLayerHeightEditMode)) vaseLayerHeightEditMode = !vaseLayerHeightEditMode;


    v.color = GuiComboBox(layoutRects[LAY_COMBO_COLORS], "YELLOW;WHITE;RED;GREEN;CYAN;MAGENTA;LILAC;GRAY", v.color);
    if (GuiValueBox(layoutRects[LAY_TEXT_RADIUS], "RADIUS: ", &v.radius, VASE_RADIUS_MIN, VASE_RADIUS_MAX, vaseRadiusEditMode)) vaseRadiusEditMode = !vaseRadiusEditMode;


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
}