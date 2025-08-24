# Real-Time Graphics Pipeline

## Overview
This project implements a **real-time graphics pipeline** from scratch in C++.  
It was developed as part of the **ISIM course** by **Thomas Galateau** and **Jules-Victor Lépinay**.  

The pipeline reproduces the main stages of a modern 3D rendering engine:
- Camera & projection (view and projection matrices)
- Model transformations
- Clipping (near/far planes, screen space boundaries)
- Rasterization (triangle drawing with color interpolation)
- Depth buffering (visibility handling)
- Multithreading support for improved performance

A sample 3D object (`teapot.obj`) is included for demonstration.

---

## Features
- **Mathematical foundations**: custom implementations of vectors (2D, 3D, 4D), matrices (4x4), and transformations.
- **Camera system**: orbital camera with rotation and configurable FOV/aspect ratio.
- **Clipping**: near/far plane clipping and screen-space clipping.
- **Rasterization**:
  - Triangle rasterization with barycentric interpolation.
  - Depth buffer for hidden surface removal.
- **Parallelism**:
  - Multi-threaded clipping and rasterization using a thread pool.
  - SIMD optimizations for triangle filling.
- **Demo object**: classic 3D teapot mesh rendering.

---

## Project Structure
.
├── CMakeLists.txt
├── src/
│ ├── Camera.cc / .hh # Camera & projection system
│ ├── Mat4x4.cc / .hh # 4x4 matrix math (transformations, projection)
│ ├── Vector2/3/4.cc / .hh # Vector operations
│ ├── Object3D.cc / .hh # 3D mesh representation
│ ├── Triangle.cc / .hh # Triangles (basic rendering unit)
│ ├── Vertex.cc / .hh # Vertex structure
│ ├── Pipeline.cc / .hh # Core graphics pipeline (clipping + rasterization)
│ ├── Engine.cc # Rendering engine entry point
│ ├── Utils.hh # Helper functions
│ ├── BS_thread_pool.hh # Thread pool for parallel execution
│ ├── teapot.obj # Demo 3D model
│ └── OpenSans-Regular.ttf # Font
└── README.md

---

## Installation & Build
This project uses **CMake** as a build system.

### Build instructions
```bash
# Clone the repository
git clone https://github.com/toma-gato/isim.git
cd isim

# Create build directory
mkdir build && cd build

# Configure and compile
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

### Run
After compilation, the executable will render the sample teapot.obj:
```bash
./MyEngine
```

---

## Camera Controls

The camera can be moved and rotated in real-time using the keyboard:

### Movement
- **Z** → Move forward  
- **S** → Move backward  
- **Q** → Move left  
- **D** → Move right  
- **R** → Move up  
- **F** → Move down  

### Rotation
- **← (Left Arrow)** → Rotate left (Y-axis)  
- **→ (Right Arrow)** → Rotate right (Y-axis)  
- **↑ (Up Arrow)** → Rotate up (X-axis)  
- **↓ (Down Arrow)** → Rotate down (X-axis)  
- **E** → Roll clockwise (Z-axis)  
- **A** → Roll counterclockwise (Z-axis)  

The camera speed and rotation are frame-rate independent, scaled by `deltaTime`.

---

## Authors
- Thomas Galateau
- Jules-Victor Lépinay

