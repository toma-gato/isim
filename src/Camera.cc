#include "Camera.hh"
#include "Vector4.hh"

Camera::Camera() : position(0.0f, 0.0f, 3.0f), target(0.0f, 0.0f, 0.0f), up(0.0f, 1.0f, 0.0f), fov(60.0f), aspectRatio(16.0f / 9.0f), nearPlane(0.1f), farPlane(100.0f), cameraAngle(0.0f), cameraRadius(7.0f), cameraHeight(2.0f) {}
Camera::Camera(Vector3 position, Vector3 target, Vector3 up, float fov, float aspectRatio, float nearPlane, float farPlane)
    : position(position), target(target), up(up), fov(fov), aspectRatio(aspectRatio), nearPlane(nearPlane), farPlane(farPlane), cameraAngle(0.0f), cameraRadius(7.0f), cameraHeight(2.0f) {}

Camera::~Camera() {}

Mat4x4 Camera::getViewMatrix() const {
    Vector3 z = (position - target).normalize();
    Vector3 x = up.cross(z).normalize();
    Vector3 y = z.cross(x).normalize();

    Mat4x4 viewMatrix;
    viewMatrix(0, 0) = x.x; viewMatrix(0, 1) = x.y; viewMatrix(0, 2) = x.z; viewMatrix(0, 3) = -x.dot(position);
    viewMatrix(1, 0) = y.x; viewMatrix(1, 1) = y.y; viewMatrix(1, 2) = y.z; viewMatrix(1, 3) = -y.dot(position);
    viewMatrix(2, 0) = z.x; viewMatrix(2, 1) = z.y; viewMatrix(2, 2) = z.z; viewMatrix(2, 3) = -z.dot(position);
    viewMatrix(3, 0) = 0.0f; viewMatrix(3, 1) = 0.0f; viewMatrix(3, 2) = 0.0f; viewMatrix(3, 3) = 1.0f;
    return viewMatrix;
}

Mat4x4 Camera::getProjectionMatrix() const {
    return Mat4x4::perspective(fov, aspectRatio, nearPlane, farPlane);
}

Mat4x4 Camera::getPositionMatrix(Mat4x4 &ModelMatrix) const {
    return this->getProjectionMatrix() * this->getViewMatrix() * ModelMatrix;
}

void Camera::updateCamera(float deltaTime)
{
    this->cameraAngle += deltaTime * 0.5f;
    
    float y = sin(this->cameraAngle) * this->cameraRadius;
    float z = cos(this->cameraAngle) * this->cameraRadius;
    
    this->position.x = this->cameraHeight;
    this->position.y = y;
    this->position.z = z;

    this->target.x = 0;
    this->target.y = 0;
    this->target.z = 0;

    this->up.x = 0;
    this->up.y = 1;
    this->up.z = 0;
}

void Camera::updateCamera(float deltaTime, float cameraAngle, float cameraRadius, float cameraHeight)
{
    cameraAngle += deltaTime * 0.5f;
    
    float y = sin(cameraAngle) * cameraRadius;
    float z = cos(cameraAngle) * cameraRadius;
    
    this->position.x = cameraHeight;
    this->position.y = y;
    this->position.z = z;

    this->target.x = 0;
    this->target.y = 0;
    this->target.z = 0;

    this->up.x = 0;
    this->up.y = 1;
    this->up.z = 0;
}

void Camera::rotateCameraY(float angle) {
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);

    Vector3 direction = target - position;
    target.x = position.x + direction.x * cosAngle - direction.z * sinAngle;
    target.z = position.z + direction.x * sinAngle + direction.z * cosAngle;
}

void Camera::rotateCameraX(float angle) {
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);

    Vector3 direction = target - position;
    target.y = position.y + direction.y * cosAngle - direction.z * sinAngle;
    target.z = position.z + direction.y * sinAngle + direction.z * cosAngle;
}

void Camera::rotateCameraZ(float angle) {
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);

    float newUpX = up.x * cosAngle - up.y * sinAngle;
    float newUpY = up.x * sinAngle + up.y * cosAngle;

    up.x = newUpX;
    up.y = newUpY;
}
