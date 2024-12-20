#include "camera.h"

Camera::Camera(glm::vec3 cameraPosition)
{
	this->cameraPosition = cameraPosition;
	this->cameraViewDirection = glm::vec3(0.0f, 0.0f, -1.0f);
	this->cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
	this->rotationOx = 0.0f;
	this->rotationOy = -90.0f;
	this->right = glm::cross(cameraViewDirection, cameraUp);
}

Camera::Camera()
{
	this ->cameraPosition = glm::vec3(0.0f, 3.0f - 30.0f, 10.0f);
	this ->cameraViewDirection = glm::vec3(0.0f, -1.0f, -1.0f);
	this ->cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
	this->rotationOx = 0.0f;
	this->rotationOy = -90.0f;
	this->right = glm::cross(cameraViewDirection, cameraUp);
}

Camera::~Camera()
{
}

void Camera::keyboardMoveFront(float cameraSpeed)
{
	cameraPosition += cameraViewDirection * cameraSpeed;
}

void Camera::keyboardMoveBack(float cameraSpeed)
{
	cameraPosition -= cameraViewDirection * cameraSpeed;
}

void Camera::keyboardMoveLeft(float cameraSpeed)
{
	cameraPosition -= glm::normalize(glm::cross(cameraViewDirection, cameraUp)) * cameraSpeed;
}

void Camera::keyboardMoveRight(float cameraSpeed)
{
	cameraPosition += glm::normalize(glm::cross(cameraViewDirection, cameraUp)) * cameraSpeed;
}

void Camera::keyboardMoveUp(float cameraSpeed)
{
	cameraPosition += cameraUp * cameraSpeed;
}

void Camera::keyboardMoveDown(float cameraSpeed)
{
	cameraPosition -= cameraUp * cameraSpeed;
}

void Camera::rotateOx(float angle)
{
	glm::vec3 cameraRight = glm::cross(cameraViewDirection, cameraUp);
	cameraViewDirection = glm::normalize(glm::vec3((glm::rotate(glm::mat4(1.0f), angle, cameraRight) * glm::vec4(cameraViewDirection, 1))));
	cameraUp = glm::normalize(glm::cross(cameraRight, cameraViewDirection));
}

void Camera::rotateOy (float angle)
{
	glm::vec3 cameraRight = glm::cross(cameraViewDirection, cameraUp);
	cameraViewDirection = glm::normalize(glm::vec3((glm::rotate(glm::mat4(1.0f), angle, cameraUp) * glm::vec4(cameraViewDirection, 1))));
	
	cameraRight = glm::normalize(glm::vec3((glm::rotate(glm::mat4(1.0f), angle, cameraUp) * glm::vec4(cameraRight, 1))));
	cameraUp = glm::normalize(glm::cross(cameraRight, cameraViewDirection));
}

void Camera::mouseMovement(float xpos, float ypos, int windowWidth, int windowHeight)
{
	float sensitivity = 0.1f;

	double xoffset = -(xpos - windowWidth / 2) * sensitivity;
	double yoffset = (windowHeight / 2 - ypos) * sensitivity;

	rotationOy += xoffset;
	rotationOx -= yoffset;

	update(xoffset, yoffset);
}

glm::mat4 Camera::getViewMatrix()
{
	return glm::lookAt(cameraPosition, cameraPosition + cameraViewDirection, cameraUp);
}

glm::vec3 Camera::getCameraPosition()
{
	return cameraPosition;
}

glm::vec3 Camera::getCameraViewDirection()
{
	return cameraViewDirection;
}

glm::vec3 Camera::getCameraUp()
{
	return cameraUp;
}

void Camera::update(float xoffset, float yoffset)
{
	glm::vec3 newViewDirection;
	newViewDirection.x = cos(glm::radians(rotationOy)) * sin(glm::radians(rotationOx));
	newViewDirection.y = sin(glm::radians(rotationOy));
	newViewDirection.z = cos(glm::radians(rotationOy)) * cos(glm::radians(rotationOx));
	
	cameraViewDirection = glm::mat3(glm::rotate(yoffset, right)) * glm::mat3(glm::rotate(xoffset, cameraUp)) * cameraViewDirection;
	right = glm::normalize(glm::cross(cameraViewDirection, cameraUp));
}

