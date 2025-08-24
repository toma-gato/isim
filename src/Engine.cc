#include <SDL3/SDL_pixels.h>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <ostream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <assert.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_time.h>
#include <SDL3/SDL_keycode.h>
#include <SDL3_ttf/SDL_ttf.h>

#include "Object3D.hh"
#include "Vector2.hh"
#include "Vector3.hh"
#include "Vector4.hh"
#include "Mat4x4.hh"
#include "Camera.hh"
#include "Pipeline.hh"

#define DEG2RAD(x) ((x) * M_PI / 180.0f)

bool start = true;
bool Z_Key = false;
bool S_Key = false;
bool Q_Key = false;
bool D_Key = false;
bool E_Key = false;
bool A_Key = false;
bool R_Key = false;
bool F_Key = false;
bool Left_Key = false;
bool Right_Key = false;
bool Up_Key = false;
bool Down_Key = false;

Camera updateCamera(Camera &Cam, float deltaTime) {
    float speed = 10.0f * deltaTime;

    Vector3 forward = (Cam.target - Cam.position).normalize();
    Vector3 right = forward.cross(Cam.up).normalize();
    Vector3 up = right.cross(forward).normalize();

    if (Z_Key) {
        Cam.position = Cam.position + (forward * speed);
        Cam.target = Cam.target + (forward * speed);
    }
    if (S_Key) {
        Cam.position = Cam.position - (forward * speed);
        Cam.target = Cam.target - (forward * speed);
    }
    if (Q_Key) {
        Cam.position = Cam.position - (right * speed);
        Cam.target = Cam.target - (right * speed);
    }
    if (D_Key) {
        Cam.position = Cam.position + (right * speed);
        Cam.target = Cam.target + (right * speed);
    }
    if (R_Key) {
        Cam.position = Cam.position + (up * speed);
        Cam.target = Cam.target + (up * speed);
    }
    if (F_Key) {
        Cam.position = Cam.position - (up * speed);
        Cam.target = Cam.target - (up * speed);
    }


    if (Left_Key) {
        Cam.rotateCameraY(-DEG2RAD(90.0f * deltaTime));
    }
    if (Right_Key) {
        Cam.rotateCameraY(DEG2RAD(90.0f * deltaTime));
    }
    if (Up_Key) {
        Cam.rotateCameraX(-DEG2RAD(90.0f * deltaTime));
    }
    if (Down_Key) {
        Cam.rotateCameraX(DEG2RAD(90.0f * deltaTime));
    }
    if (E_Key) {
        Cam.rotateCameraZ(-DEG2RAD(90.0f * deltaTime));
    }
    if (A_Key) {
        Cam.rotateCameraZ(DEG2RAD(90.0f * deltaTime));
    }

    return Cam;
}

Object3D parseOBJ(const std::string& filename) {
    std::ifstream inFile(filename);  
    if (!inFile) {
        throw std::runtime_error("Impossible d'ouvrir le fichier OBJ: " + filename);
    }

    std::vector<Vector3> positions;
    std::vector<Vector3> normals;
    std::vector<std::array<int,3>> faces;
    std::string line;
    Object3D object;

    while (std::getline(inFile, line)) {
        if (line.empty() || line[0] == '#')
            continue;

        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            Vector3 p;
            iss >> p.x >> p.y >> p.z;
            positions.emplace_back(p);
        } else if (prefix == "f") {
            std::array<int,3> idx;
            for (int i = 0; i < 3; ++i) {
                iss >> idx[i];
                idx[i] -= 1;
            }
            faces.emplace_back(idx);
        }
        // } else if (prefix == "vn") {
        //     Vector3 n;
        //     iss >> n.x >> n.y >> n.z;
        //     normals.push_back(n);
        // }
        // else if (prefix == "f") {
        //     std::string vertexStr;
        //     std::array<int, 3> posIndices;
        //     std::array<int, 3> normIndices;
        
        //     for (int i = 0; i < 3; ++i) {
        //         iss >> vertexStr;
        //         size_t pos = vertexStr.find("//");
        //         if (pos != std::string::npos) {
        //             posIndices[i] = std::stoi(vertexStr.substr(0, pos)) - 1;
        //             normIndices[i] = std::stoi(vertexStr.substr(pos + 2)) - 1;
        //         }
        //     }
        //     // On ajoute la face :contentReference[oaicite:10]{index=10}
        //     faces.push_back({posIndices[0], posIndices[1], posIndices[2]});
        // }
    }

    for (auto const& f : faces) {
        // Récupération des positions
        Vector3 p0 = positions[f[0]];
        Vector3 p1 = positions[f[1]];
        Vector3 p2 = positions[f[2]];
        // Récupération des normales
        Vector3 n0 = {0.0f,0.0f,1.0f}; //normals[f[0]];
        Vector3 n1 = {0.0f,0.0f,1.0f}; //normals[f[1]];
        Vector3 n2 = {0.0f,0.0f,1.0f}; //normals[f[2]];

        float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

        // Création de Vertex avec normale et UV par défaut
        Vertex v0{ p0, n0, {0.0f, 0.0f}, {r,g,b,1} };
        Vertex v1{ p1, n1, {0.0f, 0.0f}, {b,r,g,1} };
        Vertex v2{ p2, n2, {0.0f, 0.0f}, {g,b,r,1} };

        // Assemblage du triangle
        Triangle tri(v0, v1, v2);
        object.addTriangle(tri);
    }
    return object;
}

int main(int argc, char** argv) {
    const int width = 1200;
    const int height = 675;

    std::vector<uint8_t> framebuffer(width * height * 4); // RGB format
    std::vector<float> depthbuffer(width * height, 255.0f);

    // Load OBJ file if provided
    Object3D model;
    if (argc > 1) {
        const std::string filename = argv[1];
        try {
            model = parseOBJ(filename);
        }
        catch (const std::exception& e) {
            std::cerr << "Erreur : " << e.what() << "\n";
            return EXIT_FAILURE;
        }
    }

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << "\n";
        return EXIT_FAILURE;
    }

    TTF_Init();
    TTF_Font* font = TTF_OpenFont("../../src/OpenSans-Regular.ttf", 24);
    if (!font) {
        std::cerr << "Erreur chargement police : " << std::endl;
        return 1;
    }

    SDL_Color textColor = {255, 255, 255};

    // Create SDL window
    SDL_Window* window = SDL_CreateWindow("3D Engine", width, height, SDL_WINDOW_RESIZABLE);
    if (!window) {
        std::cerr << "Window could not be created! SDL Error: " << SDL_GetError() << "\n";
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Create renderer
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL Error: " << SDL_GetError() << "\n";
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Create texture (matches our RGB framebuffer format)
    SDL_Texture* texture = SDL_CreateTexture(renderer, 
                                           SDL_PIXELFORMAT_RGBX8888, 
                                           SDL_TEXTUREACCESS_STREAMING, 
                                           width, height);
    if (!texture) {
        std::cerr << "Texture could not be created! SDL Error: " << SDL_GetError() << "\n";
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }

    // Create camera
    Vector3 cameraPos(0, 0, 7);
    Vector3 target(0, 0, 0);
    Vector3 up(0, 1, 0);
    float fov = 60.0f;
    float aspectRatio = static_cast<float>(width) / height;
    float nearPlane = 5.0f;
    float farPlane = 9.0f;
    //float nearPlane = 2.0f;
    //float farPlane = 10.0f;
    
    Camera cam(cameraPos, target, up, fov, aspectRatio, nearPlane, farPlane);

    
    std::vector<Object3D> sceneObjects;
    if (model.triangles.size() > 0) {
        model.modelMatrix = model.modelMatrix.translation(0.0f, -3.0f, 0.0f);
        sceneObjects.push_back(model);
        model.modelMatrix = model.modelMatrix.rotationX(3.14159);
        model.modelMatrix = model.modelMatrix * model.modelMatrix.rotationY(3.14159);
        model.modelMatrix = model.modelMatrix * model.modelMatrix.translation(0.0f, -3.5f, 0.0f);
        sceneObjects.push_back(model);
    }

    // Create pipeline
    Pipeline pipeline(cam, sceneObjects, framebuffer, height, width);

    bool quit = false;
    SDL_Event e;

    Uint64 renderedFrames = 0;

    Uint64 NOW = SDL_GetPerformanceCounter();
    Uint64 LAST = 0;
    float deltaTime = 0;

    int frames_drawn = 0;
    Uint32 fps_counter = 0;
    float fps = 0.0f;
    Uint32 prev_ticks = SDL_GetTicks();

    std::cout << SDL_GetWindowPixelFormat(window);
    // Main loop
    while (!quit) {

        LAST = NOW;
        NOW = SDL_GetPerformanceCounter();
        deltaTime = (float)((NOW - LAST) / (float)SDL_GetPerformanceFrequency());

        if (deltaTime > 0.05f) deltaTime = 0.05f;

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) {
                quit = true;
            } else if (e.type == SDL_EVENT_KEY_DOWN) {
                switch (e.key.key) {
                    case SDLK_Z: Z_Key = true; break;
                    case SDLK_S: S_Key = true; break;
                    case SDLK_Q: Q_Key = true; break;
                    case SDLK_D: D_Key = true; break;
                    case SDLK_E: E_Key = true; break;
                    case SDLK_A: A_Key = true; break;
                    case SDLK_R: R_Key = true; break;
                    case SDLK_F: F_Key = true; break;
                    case SDLK_LEFT: Left_Key = true; break;
                    case SDLK_RIGHT: Right_Key = true; break;
                    case SDLK_UP: Up_Key = true; break;
                    case SDLK_DOWN: Down_Key = true; break;
                    case SDLK_ESCAPE: quit = true; break;
                }
            } else if (e.type == SDL_EVENT_KEY_UP) {
                switch (e.key.key) {
                    case SDLK_Z: Z_Key = false; break;
                    case SDLK_S: S_Key = false; break;
                    case SDLK_Q: Q_Key = false; break;
                    case SDLK_D: D_Key = false; break;
                    case SDLK_E: E_Key = false; break;
                    case SDLK_A: A_Key = false; break;
                    case SDLK_R: R_Key = false; break;
                    case SDLK_F: F_Key = false; break;
                    case SDLK_LEFT: Left_Key = false; break;
                    case SDLK_RIGHT: Right_Key = false; break;
                    case SDLK_UP: Up_Key = false; break;
                    case SDLK_DOWN: Down_Key = false; break;
                    case SDLK_ESCAPE: quit = false; break;
                }
            }
        }

        // Pixel on my screen are XRGB, so alpha channel is in first
        pipeline.rasterizeSceneThreadedFull();
        SDL_UpdateTexture(texture, NULL, pipeline.frameBuffer.data(), width * 4);
        SDL_RenderTexture(renderer, texture, NULL, NULL);
        SDL_RenderPresent(renderer);

        pipeline.SceneObjects = sceneObjects;
        pipeline.Cam.updateCamera(deltaTime);
        
        // if (Z_Key || S_Key || Q_Key || D_Key || E_Key || A_Key || R_Key || F_Key || Left_Key || Right_Key || Up_Key || Down_Key) {
        //     pipeline.Cam = updateCamera(pipeline.Cam, deltaTime);
        // }
        
        pipeline.frameBuffer = framebuffer;
        pipeline.depthBuffer = depthbuffer;

        renderedFrames++;
        Uint32 ticks_now = SDL_GetTicks();
        Uint32 diff = ticks_now - prev_ticks;
        fps_counter += diff;
        prev_ticks = ticks_now;
        frames_drawn++;

        if(fps_counter >= 1000) {
            fps = (float)frames_drawn / ((float)fps_counter/1000.0f);
            std::cout << "fps :" << fps << std::endl;
            frames_drawn = 0;
            fps_counter = 0;
        }

        std::stringstream timeText;
        timeText << "FPS: " << static_cast<int>(fps);
        SDL_Surface* textSurface = TTF_RenderText_Solid(font, timeText.str().c_str(), timeText.str().length(), textColor);
        SDL_Texture* textTexture = SDL_CreateTextureFromSurface(renderer, textSurface);

        SDL_FRect renderQuad = {20, 20, (float)textSurface->w, (float)textSurface->h};

        SDL_RenderTexture(renderer, textTexture, NULL, &renderQuad);

        SDL_DestroySurface(textSurface);
        SDL_DestroyTexture(textTexture);

        SDL_RenderPresent(renderer);
    }
    // Cleanup
    TTF_CloseFont(font);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();

    return 0;
}
