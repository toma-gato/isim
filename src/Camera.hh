#pragma once

#include "Vector3.hh"
#include "Mat4x4.hh"
#include "Vector4.hh"
#include <vector>

class Camera {
public:
    Camera();
    Camera(Vector3 position, Vector3 target, Vector3 up, float fov, float aspectRatio, float nearPlane, float farPlane);

    ~Camera();

    Vector3 position;
    Vector3 target;
    Vector3 up;
    float fov;
    float aspectRatio;
    float nearPlane;
    float farPlane;
    float cameraAngle;
    float cameraRadius;
    float cameraHeight;

    Mat4x4 getViewMatrix() const;
    Mat4x4 getProjectionMatrix() const;
    Mat4x4 getPositionMatrix(Mat4x4 &ModelMatrix) const;
    void updateCamera(float deltaTime);
    void updateCamera(float deltaTime, float cameraAngle, float cameraRadius, float cameraHeight);
    void rotateCameraY(float angle);
    void rotateCameraX(float angle);
    void rotateCameraZ(float angle);
};
