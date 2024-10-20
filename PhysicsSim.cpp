#include "src\window\window.h"
#include "src\camera\camera.h"
#include "src\shaders\shader.h"
#include "src\mesh\mesh.h"
#include "src\mesh\texture.h"
#include "src\mesh\meshLoaderObj.h"

#include <map>

using namespace std;

void createAndLoadShaders();
void processKeyboardInput();
void processMouseInput();
void processEvents();

Window window("Physics simulation", 800, 800);
Camera camera;
MeshLoaderObj loader;
map<string, Texture> textures;

float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

glm::vec3 lightColor = glm::vec3(1.0f);
glm::vec3 lightPos = glm::vec3(-180.0f, 100.0f, -200.0f);

void createAndLoadSingleTexture(string textureName, string path, string textureType = "texture_diffuse")
{
	GLuint tex = loadBMP(path.c_str());
	Texture texture = Texture();
	texture.id = tex;
	texture.type = textureType;
	textures[textureName] = texture;
}

void createAndLoadTextures()
{
	createAndLoadSingleTexture("wood", "resources/Textures/wood.bmp");
	createAndLoadSingleTexture("rock", "resources/Textures/rock.bmp");
	createAndLoadSingleTexture("orange", "resources/Textures/orange.bmp");
}

int main()
{
	glEnable(GL_DEPTH_TEST);

	Shader shader = Shader("src/shaders/vertex_shader.glsl", "src/shaders/fragment_shader.glsl");

	createAndLoadTextures();

	MeshLoaderObj loader;
	Mesh sphere = loader.loadObj("resources/Models/sphere.obj");
	Mesh box = loader.loadObj("resources/Models/cube.obj");

	while (!window.isPressed(GLFW_KEY_ESCAPE) && glfwWindowShouldClose(window.getWindow()) == 0)
	{
		glClearColor(0, 0, 0, 1.0f);
		processEvents();

		glm::mat4 ModelMatrix = glm::mat4(1.0);
		glm::mat4 ProjectionMatrix = glm::perspective(90.0f, window.getWidth() * 1.0f / window.getHeight(), 0.1f, 10000.0f);
		glm::mat4 ViewMatrix = glm::lookAt(camera.getCameraPosition(), camera.getCameraPosition() + camera.getCameraViewDirection(), camera.getCameraUp());
		glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

		// Draw "collision box"
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		shader.use();

		GLuint MatrixID2 = glGetUniformLocation(shader.getId(), "MVP");
		GLuint ModelMatrixID = glGetUniformLocation(shader.getId(), "model");

		ModelMatrix = glm::mat4(1.0);
		ModelMatrix = glm::translate(ModelMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
		ModelMatrix = glm::scale(ModelMatrix, glm::vec3(10.0f, 10.0f, 10.0f));
		MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
		glUniformMatrix4fv(MatrixID2, 1, GL_FALSE, &MVP[0][0]);
		glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
		glUniform3f(glGetUniformLocation(shader.getId(), "lightColor"), lightColor.x, lightColor.y, lightColor.z);
		glUniform3f(glGetUniformLocation(shader.getId(), "lightPos"), lightPos.x, lightPos.y, lightPos.z);
		glUniform3f(glGetUniformLocation(shader.getId(), "viewPos"), camera.getCameraPosition().x, camera.getCameraPosition().y, camera.getCameraPosition().z);

		box.draw(shader);

		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// Draw objects inside said collision box

		for (int i = 0; i < 1; i++)
		{
			ModelMatrix = glm::mat4(1.0);
			ModelMatrix = glm::translate(ModelMatrix, glm::vec3(0.0f, 0.0f, 0.0f));
			MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
			glUniformMatrix4fv(MatrixID2, 1, GL_FALSE, &MVP[0][0]);
			glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
			glUniform3f(glGetUniformLocation(shader.getId(), "lightColor"), lightColor.x, lightColor.y, lightColor.z);
			glUniform3f(glGetUniformLocation(shader.getId(), "lightPos"), lightPos.x, lightPos.y, lightPos.z);
			glUniform3f(glGetUniformLocation(shader.getId(), "viewPos"), camera.getCameraPosition().x, camera.getCameraPosition().y, camera.getCameraPosition().z);
			box.draw(shader);
		}

		window.update();
	}
}

void processEvents()
{
	window.clear();
	float currentFrame = glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;

	processKeyboardInput();
	processMouseInput();
}

void processMouseInput()
{

}

void processKeyboardInput()
{
	float cameraSpeed = 30 * deltaTime;

	//translation
	if (window.isPressed(GLFW_KEY_W))
		camera.keyboardMoveFront(cameraSpeed);
	if (window.isPressed(GLFW_KEY_S))
		camera.keyboardMoveBack(cameraSpeed);
	if (window.isPressed(GLFW_KEY_A))
		camera.keyboardMoveLeft(cameraSpeed);
	if (window.isPressed(GLFW_KEY_D))
		camera.keyboardMoveRight(cameraSpeed);
	if (window.isPressed(GLFW_KEY_R))
		camera.keyboardMoveUp(cameraSpeed);
	if (window.isPressed(GLFW_KEY_F))
		camera.keyboardMoveDown(cameraSpeed);

	//rotation
	if (window.isPressed(GLFW_KEY_LEFT))
		camera.rotateOy(cameraSpeed);
	if (window.isPressed(GLFW_KEY_RIGHT))
		camera.rotateOy(-cameraSpeed);
	if (window.isPressed(GLFW_KEY_UP))
		camera.rotateOx(cameraSpeed);
	if (window.isPressed(GLFW_KEY_DOWN))
		camera.rotateOx(-cameraSpeed);
}
