#include "raylib.h"
#include "raymath.h"

#include "raygui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CALC_NORMALS_WHEN_GENERATE false

#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"

#include "generator.h"


int screen_w = 640, screen_h = 480;                 // Current resolution
int required_screen_w = 0, required_screen_h = 0;   // Required resolution (resize event!)

uint16_t globalNoiseId = 1;
float genTiming = 0;

Vector2 ancSettings = (Vector2){50, 168 };
Vector2 ancNoise    = (Vector2){40, 168 + 40};

// WASM
void emscripten_run_script(const char *script);
void emscripten_set_main_loop(void (*func)(), int fps, int simulate_infinite_loop);

enum {
    LAY_WINDOW_SETTINGS = 0,
    LAY_WINDOW_NOISE,

    // SETTINGS
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

    // NOISE
    LAY_DROPDOWN_NOISE,
    LAY_BUTTON_ADD_NOISE,
    LAY_BUTTON_REMOVE_NOISE,
    LAY_CHECKBOX_NOISE_VISIBILE,
    LAY_TEXT_XSCALE,
    LAY_TEXT_YSCALE,
    LAY_TEXT_TWIST,
    LAY_NOISE_STRENGTH_GROUP,
    LAY_NOISE_STRENGTH,
    LAY_SCROLL_STRENGTH_0, LAY_SCROLL_STRENGTH_1, LAY_SCROLL_STRENGTH_2, LAY_SCROLL_STRENGTH_3, LAY_SCROLL_STRENGTH_4,
    LAY_SCROLL_STRENGTH_5, LAY_SCROLL_STRENGTH_6, LAY_SCROLL_STRENGTH_7, LAY_SCROLL_STRENGTH_8, LAY_SCROLL_STRENGTH_9,
    LAY_COUNT
} LayoutRects;

Camera  camera;
Shader  shader;
Light   light;
Model   model;
Vase    vase;

// Used to manage popups
int32_t     hittedWindow = INT32_MIN;
int32_t     activeWindow = INT32_MIN;
int32_t     draggedWindow = INT32_MIN;
int32_t     zOrder[2] = {LAY_WINDOW_SETTINGS, LAY_WINDOW_NOISE};

bool isWindowVisible[2] = {false, false};

// Main menu
Texture icons[4][2];
const char* iconsDescr[4] = { "NEW VASE", "DOWNLOAD THIS VASE", "VASE SETTINGS", "NOISE SETTINGS"};

// Refresh button
Texture refreshIcon;

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

// Some default values
#define VASE_RADIUS_MIN 10
#define VASE_RADIUS_MAX 80

#define VASE_HEIGHT_MIN 1
#define VASE_HEIGHT_MAX 200

#define VASE_RESOLUTION_MIN 4
#define VASE_RESOLUTION_MAX 500

#define VASE_LAYER_HEIGHT_MIN 50
#define VASE_LAYER_HEIGHT_MAX 200*1000

#define VASE_MAX_PROFILE_DEPTH 50

// Forward decls
void updateFrame();
void initGui();
void renderGui();
bool isEditing();

void onButtonNew();
void onButtonExport();

bool smoothModel = false;
bool autoRegenerate = true;

// Default gui font
Font font;

// Called from JS on resize
void resize(int w, int h)
{
    required_screen_w = w;
    required_screen_h = h;
}

// Re-init vase
void initVase()
{
    globalNoiseId = 1;
    vase.toUpdate = true;
    vase.height = 100;
    vase.radius = 20;
    vase.resolution = 300;
    vase.layerHeight = 200;
    vase.color = 0;

    vase.profileDepth = 10;
    for(size_t i = 0; i< 10; i++)
        vase.profile[i] = GetRandomValue(0,1000);

    for(size_t i = 0; i<MAX_NOISES; ++i)
    {
        vase.noise[i].enabled = false;
        vase.noise[i].visible = true;

        for(size_t k = 0; k < 10; ++k)
            vase.noise[i].alpha[k] = 1.0;

        vase.noise[i].direction = (Vector3){1,0,1};
        vase.noise[i].height = 20;
        vase.noise[i].seed = GetRandomValue(-1000000000, 1000000000);
    }
}

int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT | FLAG_MSAA_4X_HINT);
    InitWindow(screen_w, screen_h, "Vasaro");
    resize(screen_w, screen_h);

    // Gui Style
    GuiLoadStyle("resources/style.rgs");

    initVase();
    initGui();

    // Font used by left menu
    font =  LoadFontEx("resources/Zector.ttf", 20, NULL, 0);

    // Icons used by left menu
    icons[0][1] = LoadTexture("resources/icons/new.empty.png");
    icons[0][0] = LoadTexture("resources/icons/new.png");

    icons[1][0] = LoadTexture("resources/icons/download.png");
    icons[1][1] = LoadTexture("resources/icons/download.empty.png");

    icons[2][0] = LoadTexture("resources/icons/vase.png");
    icons[2][1] = LoadTexture("resources/icons/vase.empty.png");

    icons[3][0] = LoadTexture("resources/icons/layers.png");
    icons[3][1] = LoadTexture("resources/icons/layers.empty.png");

    refreshIcon = LoadTexture("resources/icons/refresh.png");

    // Init camera
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

    // Let's generate the first vase
    genTiming = regenerate(smoothModel);

    emscripten_run_script("update_canvas_size();");
    emscripten_set_main_loop(updateFrame, 0, 1);

    CloseWindow();        // Close window and OpenGL context
    return 0;
}

bool isMouseInputValidFor(int32_t window) { return activeWindow == INT32_MIN || activeWindow == window; }

void updateFrame()
{

    // First check: resize window if needed
    if (IsWindowResized() || required_screen_w != screen_w ||required_screen_h != screen_h)
    {
        screen_w = required_screen_w;
        screen_h = required_screen_h;
        SetWindowSize(screen_w, screen_h);
        initGui();
    }

    static float modelRotationX = 45;
    static float cameraHeight = 150;

    static bool isOverGUI = false;
    static bool wasButtonDown = false;

    if (vase.toUpdate && autoRegenerate && IsMouseButtonUp(0))
        genTiming = regenerate(smoothModel);

    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonUp(0))
    {
        activeWindow = INT32_MIN;
        draggedWindow = INT32_MIN;
    }

    hittedWindow = INT32_MIN;

    if (CheckCollisionPointRec(mouse, (Rectangle){0,0,48,screen_h}))
        hittedWindow = -1;
    else
    {
        for(int i = 0; i < 2; ++i)
        {
            if (isWindowVisible[zOrder[i]] && CheckCollisionPointRec(mouse, layoutRects[zOrder[i]]))
            {
                hittedWindow = zOrder[i];
                break;
            }
        }
    }

    if (activeWindow == INT32_MIN)
    {
        if (IsMouseButtonDown(0) && !wasButtonDown)
        {
            activeWindow = hittedWindow;

            if (activeWindow == LAY_WINDOW_SETTINGS)
            {
                zOrder[0] = LAY_WINDOW_SETTINGS;
                zOrder[1] = LAY_WINDOW_NOISE;
            }
            else if (activeWindow == LAY_WINDOW_NOISE)
            {
                zOrder[1] = LAY_WINDOW_SETTINGS;
                zOrder[0] = LAY_WINDOW_NOISE;
            }

            if (activeWindow == INT32_MIN)
                activeWindow = -2;

            if (hittedWindow >= 0)
            {
                Rectangle header = layoutRects[hittedWindow];
                header.height = 24;

                if (CheckCollisionPointRec(mouse, header))
                {
                    draggedWindow = hittedWindow;
                }

            }
        }
    }

    if (IsMouseButtonDown(0) && draggedWindow >= 0)
    {
        if (draggedWindow == LAY_WINDOW_SETTINGS)
        {
            ancSettings.x += GetMouseDelta().x;
            ancSettings.y += GetMouseDelta().y;
            initGui();
        }
        else if (draggedWindow == LAY_WINDOW_NOISE)
        {
            ancNoise.x += GetMouseDelta().x;
            ancNoise.y += GetMouseDelta().y;
            initGui();
        }
    }

    isOverGUI = activeWindow >= 0;

    // Manual regeneration click
    if (!autoRegenerate && vase.toUpdate &&
        !isOverGUI && !wasButtonDown && IsMouseButtonDown(0)
        && mouse.x > screen_w - 32- 20 && mouse.x < screen_w - 20
        && mouse.y > 20 && mouse.y < 20 + 32
        )
        genTiming = regenerate(smoothModel);

    // Move model
    {
        static float speedX = 0;
        if (IsMouseButtonDown(0) && activeWindow < -1)
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

    // Update view
    {
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

            //DrawModelEx(model, (Vector3){0,0,0}, (Vector3){0,1,0}, 0, (Vector3){1,1,1}, colors[vase.color]);
            DrawModel(model, (Vector3){0,0,0}, 1, colors[vase.color]);
            EndMode3D();
        }

        if (vase.toUpdate && !autoRegenerate)
            DrawTexture(refreshIcon, screen_w - 32 - 20, 20, (Color){200,200,200,100});

        // Draw UI

        renderGui();

        Color c = {100,100,100,200};
        Color c1 = {180,180,180,200};

        DrawRectangle(0,0,48,screen_h, (Color){15,36,51,230});

        for(int i = 0; i < 4; ++i)
        {
            if (isMouseInputValidFor(-1) && CheckCollisionPointRec(mouse, (Rectangle){8,8*(i+1)+40*i,32,32}))
            {
                DrawTexture(icons[i][0],8,8*(i+1)+40*i,c1);
                DrawTextPro(font, iconsDescr[i], (Vector2) {33, 210}, (Vector2) {0, 0}, 90, 20, 3, (Color){180,180,180,200});

                if(IsMouseButtonPressed(0))
                {
                    switch(i)
                    {
                        case 0:
                            onButtonNew();
                            break;

                        case 1:
                            onButtonExport();
                            break;

                        case 2:
                            zOrder[0] = LAY_WINDOW_SETTINGS;
                            zOrder[1] = LAY_WINDOW_NOISE;

                            isWindowVisible[LAY_WINDOW_SETTINGS] = true;
                            ancSettings.x = 50;
                            ancSettings.y = 8*(i+1)+40*i;
                            initGui();
                            break;

                        case 3:
                            zOrder[1] = LAY_WINDOW_SETTINGS;
                            zOrder[0] = LAY_WINDOW_NOISE;

                            isWindowVisible[LAY_WINDOW_NOISE] = true;
                            ancNoise.x = 50;
                            ancNoise.y = 8*(i+1)+40*i;
                            initGui();
                            break;
                    }
                }


            }
            else
                DrawTexture(icons[i][1],8,8*(i+1)+40*i,c);
        }


        int noiseCount = 0;

        for(int i = 0; i < MAX_NOISES; i++)
        {
            if (vase.noise[i].enabled == true) noiseCount++;
            else break;
        }

        if (noiseCount > 0)
        {
            char countText[2] = {0};
            snprintf(countText, 2, "%d", noiseCount);
            DrawCircle(35, 8*3+40*3+32, 8, (Color){150,20,20,220});
            DrawText(countText, 35-3,8*3+40*3+32-4, 10, RAYWHITE );
        }

        static bool lastEditingState = false;

        if (lastEditingState != isEditing())
        {
            lastEditingState = isEditing();
            if (lastEditingState == false) vase.toUpdate = true;
        }

        char stats[200];
        snprintf(stats, 200, "%d TRIANGLES GENERATED IN %0.3fs", model.meshes[0].triangleCount, genTiming);
        DrawText(stats, 60, screen_h - 20, 10, (Color){230,230,230,200});
    EndDrawing();


    wasButtonDown = IsMouseButtonDown(0);

    //----------------------------------------------------------------------------------
}



void onButtonExport()
{
    if (vase.toUpdate)
        genTiming = regenerate(smoothModel);

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
    // SETTINGS
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
    layoutRects[LAY_COMBO_COLORS] = (Rectangle){ ancRendering.x + 5,  ancRendering.y + 10, 130, 24 };
    layoutRects[LAY_SMOOTH] = (Rectangle){ ancRendering.x + 5,  ancRendering.y + 10 + 40, 130, 24 };
    layoutRects[LAY_AUTOREGEN] = (Rectangle){ ancRendering.x + 5,  ancRendering.y + 10 + 40*2, 130, 24 };

    // NOISE
    layoutRects[LAY_WINDOW_NOISE] = (Rectangle){ ancNoise.x + 8, ancNoise.y + 0, 360, 435 };
    layoutRects[LAY_DROPDOWN_NOISE] = (Rectangle) { ancNoise.x + 10 + 8, ancNoise.y + 8 + 24 + 8, 160, 24};
    layoutRects[LAY_BUTTON_ADD_NOISE] = (Rectangle){ ancNoise.x + 10 + 160 + 8*2 , ancNoise.y + 8 + 24 + 8, 80, 24};
    layoutRects[LAY_BUTTON_REMOVE_NOISE] = (Rectangle){ ancNoise.x + 10 + 160 + 8*3 + 80 , ancNoise.y + 8 + 24 + 8, 80, 24};

}

void onButtonNew()
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

bool noiseDropdownEditMode = false;

bool isEditing()
{
   return vaseProfileDepthEditMode || vaseHeightEditMode || vaseLayerHeightEditMode || vasePointsEditMode || vaseRadiusEditMode;
}


void renderNoiseWindow()
{
    static int selectedNoise = 0;

    if (!isWindowVisible[LAY_WINDOW_NOISE]) return;

    if (!isMouseInputValidFor(LAY_WINDOW_NOISE) || hittedWindow != LAY_WINDOW_NOISE) GuiLock();
    else GuiUnlock();


    isWindowVisible[LAY_WINDOW_NOISE] = !GuiWindowBox(layoutRects[LAY_WINDOW_NOISE], "NOISE SETTINGS");


    int noiseCount = 0;
    bool hasNoise =  (vase.noise[0].enabled);

    char items[(10+7)*8] = {0};

    if (hasNoise)
    {
        int pos = 0;
        for(int i = 0; i < MAX_NOISES; i++)
        {
            if (vase.noise[i].enabled == true)
            {
                noiseCount++;
                pos += snprintf(&items[pos], 80, "NOISE #%zu;", vase.noise[i].noiseId);
            }
            else break;
        }

        pos += snprintf(&items[pos], 80, "<NOTHING>");

    }

    if (hasNoise) GuiEnable();
    else GuiDisable();

    if (GuiDropdownBox(layoutRects[LAY_DROPDOWN_NOISE], items, &selectedNoise, noiseDropdownEditMode) ) noiseDropdownEditMode = !noiseDropdownEditMode;



    if (noiseCount < 8) GuiEnable();
    else GuiDisable();
    if (GuiButton(layoutRects[LAY_BUTTON_ADD_NOISE], "ADD")) {
        vase.noise[noiseCount].enabled = true;
        vase.noise[noiseCount].height = 5;
        vase.noise[noiseCount].seed = GetRandomValue(0, INT32_MAX);
        vase.noise[noiseCount].noiseId = globalNoiseId++;
        selectedNoise = noiseCount;
        vase.toUpdate = true;
    }

    if (noiseCount > 0 && selectedNoise < noiseCount) GuiEnable();
    else GuiDisable();
    if (GuiButton(layoutRects[LAY_BUTTON_REMOVE_NOISE], "REMOVE")) {

        if (selectedNoise < MAX_NOISES - 1)
            memmove(&vase.noise[selectedNoise], &vase.noise[selectedNoise+1], (MAX_NOISES - selectedNoise - 1)*sizeof(Noise));

        vase.noise[MAX_NOISES - 1].enabled = false;

        selectedNoise = noiseCount-1;
        vase.toUpdate = true;


    }

    GuiEnable();
}

void renderSettingsWindow()
{
    if (!isWindowVisible[LAY_WINDOW_SETTINGS]) return;

    static bool profileChanged = false;

    if (!isMouseInputValidFor(LAY_WINDOW_SETTINGS) || hittedWindow != LAY_WINDOW_SETTINGS) GuiLock();
    else GuiUnlock();


    isWindowVisible[LAY_WINDOW_SETTINGS] = !GuiWindowBox(layoutRects[LAY_WINDOW_SETTINGS], "VASE SETTINGS");
    if (GuiValueBox(layoutRects[LAY_TEXT_RADIUS], "RADIUS (MM) ", &vase.radius, VASE_RADIUS_MIN, VASE_RADIUS_MAX, vaseRadiusEditMode)) vaseRadiusEditMode = !vaseRadiusEditMode;
    if (GuiValueBox(layoutRects[LAY_TEXT_HEIGHT], "HEIGHT (MM) ", &vase.height, VASE_HEIGHT_MIN, VASE_HEIGHT_MAX, vaseHeightEditMode)) vaseHeightEditMode = !vaseHeightEditMode;
    if (GuiValueBox(layoutRects[LAY_TEXT_POINTS], "BASE VERTICES ", &vase.resolution, VASE_RESOLUTION_MIN, VASE_RESOLUTION_MAX, vasePointsEditMode)) vasePointsEditMode = !vasePointsEditMode;

    int maxLayers = 1000*vase.height;
    if (maxLayers < VASE_LAYER_HEIGHT_MIN) maxLayers = vase.layerHeight;
    if (GuiValueBox(layoutRects[LAY_TEXT_LAYERH], "LAYER (MICRON) ", &vase.layerHeight, VASE_LAYER_HEIGHT_MIN, maxLayers, vaseLayerHeightEditMode)) vaseLayerHeightEditMode = !vaseLayerHeightEditMode;


    GuiGroupBox(layoutRects[LAY_PROFILE_GROUP], "PROFILE");
    if (GuiValueBox(layoutRects[LAY_BUTTON_PROFILE_THICKNESS], "DEPTH ", &vase.profileDepth, 0, VASE_MAX_PROFILE_DEPTH, vaseProfileDepthEditMode)) vaseProfileDepthEditMode = !vaseProfileDepthEditMode;

    bool isStillOver = false;
    for(size_t i = 0; i < 10; i++)
    {
        int32_t old = vase.profile[i];
        vase.profile[i] = GuiSlider(layoutRects[LAY_SCROLL_PROFILE_0 + i], "", "", vase.profile[i], 0, 1000);
        if (old != vase.profile[i]) profileChanged = true;

        if(!isStillOver && CheckCollisionPointRec(GetMousePosition(), layoutRects[LAY_SCROLL_PROFILE_0 +i]) && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            isStillOver = true;
    }

    GuiGroupBox(layoutRects[LAY_RENDERING_GROUP], "RENDERING");
    vase.color = GuiComboBox(layoutRects[LAY_COMBO_COLORS], "YELLOW;WHITE;RED;GREEN;CYAN;MAGENTA;LILAC;GRAY", vase.color);

    bool wasSmooth = smoothModel;
    smoothModel = GuiToggle(layoutRects[LAY_SMOOTH], "SMOOTH MODEL", smoothModel);
    if (wasSmooth != smoothModel) vase.toUpdate = true;

    autoRegenerate = GuiToggle(layoutRects[LAY_AUTOREGEN], "AUTO REGENERATE", autoRegenerate);


    if (!isStillOver && profileChanged)
    {
        vase.toUpdate = true;
        profileChanged = false;
    }


    if (!isEditing())
    {
        if (vase.radius < VASE_RADIUS_MIN) vase.radius = VASE_RADIUS_MIN;
        if (vase.radius > VASE_RADIUS_MAX) vase.radius = VASE_RADIUS_MAX;

        if (vase.layerHeight < VASE_LAYER_HEIGHT_MIN) vase.layerHeight = VASE_LAYER_HEIGHT_MIN;
        if (vase.layerHeight > VASE_LAYER_HEIGHT_MAX) vase.layerHeight = VASE_LAYER_HEIGHT_MAX;

        if (vase.height < VASE_HEIGHT_MIN) vase.height = VASE_HEIGHT_MIN;
        if (vase.height > VASE_HEIGHT_MAX) vase.height = VASE_HEIGHT_MAX;

        if (vase.resolution < VASE_RESOLUTION_MIN) vase.resolution = VASE_RESOLUTION_MIN;
        if (vase.resolution > VASE_RESOLUTION_MAX) vase.resolution = VASE_RESOLUTION_MAX;

        if (vase.profileDepth > VASE_MAX_PROFILE_DEPTH) vase.profileDepth = VASE_MAX_PROFILE_DEPTH;
        if (vase.profileDepth < 0) vase.profileDepth = 0;
    }
}

void renderGui()
{
    if (zOrder[0] == LAY_WINDOW_SETTINGS)
    {
        renderNoiseWindow();
        renderSettingsWindow();
    }
    else
    {
        renderSettingsWindow();
        renderNoiseWindow();
    }
}