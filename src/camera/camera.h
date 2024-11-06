#pragma once

#include <glm.hpp>
#include <gtx\transform.hpp>
#include <gtc\matrix_transform.hpp>
#include <gtc\type_ptr.hpp>
#include "..\window\window.h"

class Camera
{
	private:
		glm::vec3 cameraPosition;
		glm::vec3 cameraViewDirection;
		glm::vec3 cameraUp;
		glm::vec3 right;

		float rotationOx;
		float rotationOy;

	public:
		Camera();
		Camera(glm::vec3 cameraPosition);
		~Camera();
		glm::mat4 getViewMatrix();
		glm::vec3 getCameraPosition();
		glm::vec3 getCameraViewDirection();
		glm::vec3 getCameraUp();

		void keyboardMoveFront(float cameraSpeed);
		void keyboardMoveBack(float cameraSpeed);
		void keyboardMoveLeft(float cameraSpeed);
		void keyboardMoveRight(float cameraSpeed);
		void keyboardMoveUp(float cameraSpeed);
		void keyboardMoveDown(float cameraSpeed);

		void rotateOx(float angle);
		void rotateOy(float angle);

		void mouseMovement(float xpos, float ypos, int windowWidth, int windowHeight);
		void update(float xoffset, float yoffset);
};

