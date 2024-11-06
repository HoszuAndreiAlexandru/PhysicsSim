#include "src\window\window.h"
#include "src\camera\camera.h"
#include "src\shaders\shader.h"
#include "src\mesh\mesh.h"
#include "src\mesh\texture.h"
#include "src\mesh\meshLoaderObj.h"
#include <map>

enum MeshType
{
	BOX,
	SPHERE,
	CYLINDER
};

class GameObject
{
	public:
		glm::vec3 position;
		Texture texture;
		Mesh model;
		glm::vec3 colour;
		MeshType meshType;
		double scale = 1;
		glm::vec3 rotation = glm::vec3(0);
		glm::vec3 velocity = glm::vec3(0.0f);
		glm::vec3 acceleration = glm::vec3(0.0f);
		glm::vec3 halfExtents = glm::vec3(1.0f);
		glm::vec3 angularVelocity = glm::vec3(0.0f);
		glm::vec3 angularAcceleration = glm::vec3(0.0f);

		GameObject(){}
		~GameObject(){}

		void draw(Shader& shader)
		{
			this->model.draw(shader);
		}
};

using namespace std;

void processKeyboardInput();
void processMouseInput(GLFWwindow* window, double xpos, double ypos);
void processEvents();

double lastTime = 0;
double currentFrameP = 0;
double lastFrameTime = 0;
double FPS = 0;

int windowHeight = 800;
int windowWidth = 800;
float sensitivity = 1;
float yaw = 0;
float pitch = 0;
double posX = 0;
double posY = 0;
Window window("Physics simulation, FPS: 0", windowHeight, windowWidth);
Shader shader = Shader("src/shaders/vertex_shader.glsl", "src/shaders/fragment_shader.glsl");
Camera camera;

MeshLoaderObj loader;
Mesh sphere = loader.loadObj("resources/Models/sphere2.obj");
Mesh box = loader.loadObj("resources/Models/cube.obj");
Mesh cylinder = loader.loadObj("resources/Models/cilindru.obj");

GameObject sceneBox;
const glm::vec3 BOUNDARY_MIN(-60.0f, -60.0f, -60.0f); // Minimum corner (e.g., ground level as y = 0)
const glm::vec3 BOUNDARY_MAX(60.0f, 60.0f, 60.0f); // Maximum corner
std::vector<GameObject> objects;
map<string, Texture> textures;

float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

glm::vec3 lightColor = glm::vec3(1.0f);
glm::vec3 lightPos = glm::vec3(-180.0f, 100.0f, -200.0f);

glm::mat4 ProjectionMatrix;
glm::mat4 ViewMatrix;

// Constants for physics
const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
//const float damping = 0.999f;
const float damping = 1;

float randomFloatUnitClamped()
{
	return ((float)rand()) / (float)(RAND_MAX);
}

float randomFloatColor()
{
	return (randomFloatUnitClamped() + 0.2f) / 1.2f;
}

void calculatePerformanceMetrics()
{
	double currentTime = glfwGetTime();
	const double delta = currentTime - lastTime;
	deltaTime = currentTime - lastFrameTime;
	lastFrameTime = currentTime;
	currentFrameP++;
	if (delta >= 1.0)
	{
		FPS = double(currentFrameP) / delta;

		ostringstream s;
		s << FPS;
		string str = "Physics simulation, FPS: " + s.str() + " ; Objects: " + std::to_string(objects.size());
		//char* cstr = str.data();
		//std::cout << str << std::endl;

		glfwSetWindowTitle(window.getWindow(), str.c_str());

		currentFrameP = 0;
		lastTime = currentTime;
		//deltaTime = lastTime;
	}
}

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
	createAndLoadSingleTexture("orange", "resources/Textures/orange.bmp");
}

void drawCollisionBox(Mesh& model)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	shader.use();

	GLuint MatrixID2 = glGetUniformLocation(shader.getId(), "MVP");
	GLuint ModelMatrixID = glGetUniformLocation(shader.getId(), "model");

	glm::mat4 ModelMatrix = glm::mat4(1.0);
	ModelMatrix = glm::scale(ModelMatrix, glm::vec3(sceneBox.scale));
	ModelMatrix = glm::translate(ModelMatrix, sceneBox.position);
	glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
	glUniformMatrix4fv(MatrixID2, 1, GL_FALSE, &MVP[0][0]);
	glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
	glUniform3f(glGetUniformLocation(shader.getId(), "lightColor"), lightColor.x, lightColor.y, lightColor.z);
	glUniform3f(glGetUniformLocation(shader.getId(), "lightPos"), lightPos.x, lightPos.y, lightPos.z);
	glUniform3f(glGetUniformLocation(shader.getId(), "viewPos"), camera.getCameraPosition().x, camera.getCameraPosition().y, camera.getCameraPosition().z);
	glUniform3f(glGetUniformLocation(shader.getId(), "objectColor"), sceneBox.colour.x, sceneBox.colour.y, sceneBox.colour.z);
	model.draw(shader);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

// Function to detect and handle sphere-sphere collision
bool detectAndResolveSphereCollision(GameObject& a, GameObject& b) {
	glm::vec3 delta = b.position - a.position;
	float distance = glm::length(delta);
	float combinedRadius = a.scale + b.scale;

	if (distance < combinedRadius) {
		// Calculate the collision response
		glm::vec3 collisionNormal = glm::normalize(delta);
		float penetrationDepth = combinedRadius - distance;

		// Resolve the collision by moving the objects apart
		a.position -= collisionNormal * (penetrationDepth / 2.0f);
		b.position += collisionNormal * (penetrationDepth / 2.0f);

		// Swap velocity components (simplified elastic collision)
		glm::vec3 relativeVelocity = b.velocity - a.velocity;
		float impactSpeed = glm::dot(relativeVelocity, collisionNormal);

		if (impactSpeed < 0) { // Objects are moving toward each other
			glm::vec3 impulse = collisionNormal * impactSpeed;
			a.velocity -= impulse;
			b.velocity += impulse;
		}

		return true;
	}
	return false;
}

// Function to detect and handle AABB (box-box) collision
bool detectAndResolveBoxCollision(GameObject& a, GameObject& b) {

	// Check for overlap in the x, y, and z axes
	bool overlapX = std::abs(a.position.x - b.position.x) < (a.halfExtents.x + b.halfExtents.x);
	bool overlapY = std::abs(a.position.y - b.position.y) < (a.halfExtents.y + b.halfExtents.y);
	bool overlapZ = std::abs(a.position.z - b.position.z) < (a.halfExtents.z + b.halfExtents.z);

	if (overlapX && overlapY && overlapZ) {
		//std::cout << "there is a collission" << std::endl;
		
		// Calculate the penetration depth along each axis
		float penX = (a.halfExtents.x + b.halfExtents.x) - std::abs(a.position.x - b.position.x);
		float penY = (a.halfExtents.y + b.halfExtents.y) - std::abs(a.position.y - b.position.y);
		float penZ = (a.halfExtents.z + b.halfExtents.z) - std::abs(a.position.z - b.position.z);

		// Find the axis of minimum penetration
		glm::vec3 correction(0.0f);
		float correctionFactor = 0.02f;
		if (penX < penY && penX < penZ) {
			correction.x = penX * (a.position.x < b.position.x ? -correctionFactor : correctionFactor);
		}
		if (penY < penX && penY < penZ) {
			correction.y = penY * (a.position.y < b.position.y ? -correctionFactor : correctionFactor);
		}
		if (penZ < penX && penZ < penY)
		{
			correction.z = penZ * (a.position.z < b.position.z ? -correctionFactor : correctionFactor);
		}

		// Apply the correction to both objects

		a.position += correction;
		b.position -= correction;

		// Adjust velocities (basic velocity response)
		glm::vec3 collisionNormal = correction;
		glm::vec3 relativeVelocity = b.velocity - a.velocity;
		glm::vec3 relativeAngularVelocity = b.angularVelocity - a.angularVelocity;

		float impactSpeed = glm::dot(relativeVelocity, collisionNormal);

		if (impactSpeed < 0) { // Only adjust if moving toward each other
			glm::vec3 impulse = collisionNormal * impactSpeed;
			a.velocity -= impulse;
			b.velocity += impulse;

			// Calculate and apply torque
			glm::vec3 torque1 = glm::cross(a.halfExtents, impulse);
			glm::vec3 torque2 = glm::cross(b.halfExtents, impulse);
			a.angularAcceleration += torque1;
			b.angularAcceleration += torque2;
		}

		return true;
	}
	return false;
}

void calculateCollision(GameObject& go1, GameObject& go2)
{
	if (go1.meshType == SPHERE && go2.meshType == SPHERE) {
		detectAndResolveSphereCollision(go1, go2);
	}
	else if (go1.meshType == BOX && go2.meshType == BOX) {
		detectAndResolveBoxCollision(go1, go2);
	}
}

void simulatePhysics()
{
	int i = 0;
	for (GameObject& obj : objects)
	{
		// Apply gravity to objects
		if (obj.meshType == BOX || obj.meshType == SPHERE || obj.meshType == CYLINDER) {
			obj.acceleration = gravity;
		}

		// Integrate acceleration to velocity
		obj.velocity += obj.acceleration * deltaTime;

		// Integrate angular acceleration to angular velocity
		obj.angularVelocity += obj.angularAcceleration * deltaTime;

		// Apply damping to simulate friction/resistance
		obj.velocity *= damping;
		obj.angularVelocity *= damping;

		// Integrate velocity to position
		obj.position += obj.velocity * deltaTime;

		// Integrate angular velocity to rotation (using Euler angles)
		obj.rotation += obj.angularVelocity * deltaTime;

		// Reset acceleration for the next frame
		obj.acceleration = glm::vec3(0.0f);
		obj.angularAcceleration = glm::vec3(0.0f);

		// Check for collision with the boundary box and adjust position/velocity
		if (obj.position.x - obj.scale < BOUNDARY_MIN.x) {
			obj.position.x = BOUNDARY_MIN.x + obj.scale; // Keep object inside
			obj.velocity.x = -obj.velocity.x * 0.5f; // Reverse and dampen velocity
		}
		else if (obj.position.x + obj.scale > BOUNDARY_MAX.x) {
			obj.position.x = BOUNDARY_MAX.x - obj.scale;
			obj.velocity.x = -obj.velocity.x * 0.5f;
		}

		if (obj.position.y - obj.scale < BOUNDARY_MIN.y) {
			obj.position.y = BOUNDARY_MIN.y + obj.scale;
			obj.velocity.y = -obj.velocity.y * 0.5f;
		}
		else if (obj.position.y + obj.scale > BOUNDARY_MAX.y) {
			obj.position.y = BOUNDARY_MAX.y - obj.scale;
			obj.velocity.y = -obj.velocity.y * 0.5f;
		}

		if (obj.position.z - obj.scale < BOUNDARY_MIN.z) {
			obj.position.z = BOUNDARY_MIN.z + obj.scale;
			obj.velocity.z = -obj.velocity.z * 0.5f;
		}
		else if (obj.position.z + obj.scale > BOUNDARY_MAX.z) {
			obj.position.z = BOUNDARY_MAX.z - obj.scale;
			obj.velocity.z = -obj.velocity.z * 0.5f;
		}

		// Detect and resolve collisions
		for (size_t j = i + 1; j < objects.size(); ++j)
		{
			calculateCollision(objects[i], objects[j]);
		}

		i++;
	}
}

void createSceneBox()
{
	GameObject go = GameObject();
	go.position = glm::vec3(0, 0, 0);
	go.model = box;
	go.meshType = BOX;
	go.colour = glm::vec3(1, 1, 1);
	go.scale = 30;
	objects.push_back(go);
	sceneBox = go;
}

void createObject(MeshType meshType)
{
	float distance = 30;
	GameObject go = GameObject();
	go.position = glm::vec3(randomFloatUnitClamped() * distance - distance / 2, randomFloatUnitClamped() * distance - distance / 2, randomFloatUnitClamped() * distance - distance / 2);
	go.scale = 0.5f;
	switch (meshType)
	{
	case SPHERE:
		go.model = sphere;
		break;
	case CYLINDER:
		go.model = cylinder;
		break;
	default:
		go.model = box;
		break;
	}
	go.meshType = meshType;
	go.colour = glm::vec3(randomFloatColor(), randomFloatColor(), randomFloatColor());
	objects.push_back(go);
}

void addObject(int objectType)
{
	switch (objectType)
	{
		case 0:
			// cube
			createObject(BOX);
			break;
		case 1:
			// sphere
			createObject(SPHERE);
			break;
		case 2:
			// cylinder
			createObject(CYLINDER);
			break;
		default:
			addObject(0);
			break;
	}
}

void resetScene(int sceneId)
{
	objects = std::vector<GameObject>();

	int x = 100;
	int y = 250;
	int z = 500;

	switch (sceneId)
	{
	case 1:
		// 100, 250, 500
		break;
	case 2:
		// 250, 500, 1000
		x *= 2.5f;
		y *= 2;
		z *= 2;
		break;
	case 3:
		// 500, 1000, 2500
		x *= 5;
		y *= 4;
		z *= 5;
		break;
	case 4:
		// clear scene
		break;
	default:
		resetScene(4);
		break;
	}

	if (sceneId <= 3)
	{
		for (int i = 0; i < x; i++)
		{
			addObject(BOX);
		}
		for (int i = 0; i < y; i++)
		{
			//addObject(SPHERE);
		}
		for (int i = 0; i < z; i++)
		{
			//addObject(CYLINDER);
		}
	}
}

int main()
{
	glEnable(GL_DEPTH_TEST);

	createSceneBox();
	createAndLoadTextures();
	//resetScene(1);
	glfwSetInputMode(window.getWindow(), GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
	glfwSetCursorPosCallback(window.getWindow(), processMouseInput);
	glfwSetCursorPos(window.getWindow(), windowWidth / 2, windowHeight / 2);

	glfwSetCursorPos(window.getWindow(), windowWidth / 2, windowHeight / 2);
	posX = windowWidth / 2;
	posY = windowHeight / 2;

	while (!window.isPressed(GLFW_KEY_ESCAPE) && glfwWindowShouldClose(window.getWindow()) == 0)
	{
		window.update();
		processEvents();
		calculatePerformanceMetrics();

		// Sim
		{
			simulatePhysics();
		}

		// Drawing
		{
			// Draw colission box

			drawCollisionBox(box);

			// Draw objects inside collision box

			GLuint MatrixID2 = glGetUniformLocation(shader.getId(), "MVP");
			GLuint ModelMatrixID = glGetUniformLocation(shader.getId(), "model");

			glm::mat4 ModelMatrix = glm::mat4(1.0);
			ProjectionMatrix = glm::perspective(90.0f, window.getWidth() * 1.0f / window.getHeight(), 0.1f, 10000.0f);
			ViewMatrix = glm::lookAt(camera.getCameraPosition(), camera.getCameraPosition() + camera.getCameraViewDirection(), camera.getCameraUp());
			glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

			for (int i = 0; i < objects.size(); i++)
			{
				ModelMatrix = glm::mat4(1.0);
				ModelMatrix = glm::scale(ModelMatrix, glm::vec3(objects[i].scale));
				ModelMatrix = glm::rotate(ModelMatrix, glm::radians(objects[i].rotation.x), glm::vec3(1.0f, 0.0f, 0.0f));
				ModelMatrix = glm::rotate(ModelMatrix, glm::radians(objects[i].rotation.y), glm::vec3(0.0f, 1.0f, 0.0f));
				ModelMatrix = glm::rotate(ModelMatrix, glm::radians(objects[i].rotation.z), glm::vec3(0.0f, 0.0f, 1.0f));
				ModelMatrix = glm::translate(ModelMatrix, objects[i].position);
				MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
				glUniformMatrix4fv(MatrixID2, 1, GL_FALSE, &MVP[0][0]);
				glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
				glUniform3f(glGetUniformLocation(shader.getId(), "objectColor"), objects[i].colour.x, objects[i].colour.y, objects[i].colour.z);
				glUniform3f(glGetUniformLocation(shader.getId(), "lightColor"), lightColor.x, lightColor.y, lightColor.z);
				glUniform3f(glGetUniformLocation(shader.getId(), "lightPos"), lightPos.x, lightPos.y, lightPos.z);
				glUniform3f(glGetUniformLocation(shader.getId(), "viewPos"), camera.getCameraPosition().x, camera.getCameraPosition().y, camera.getCameraPosition().z);
				objects[i].model.draw(shader);
			}
		}
	}
}

void processEvents()
{
	window.clear();
	float currentFrame = glfwGetTime();
	deltaTime = currentFrame - lastFrame;
	lastFrame = currentFrame;

	processKeyboardInput();
}

void processMouseInput(GLFWwindow* window, double xpos, double ypos)
{
	glfwSetCursorPos(window, windowWidth / 2, windowHeight / 2);
	camera.mouseMovement(xpos, ypos, windowWidth, windowHeight);
}

void processKeyboardInput()
{
	float cameraSpeed = 10 * deltaTime;

	// Camera movement
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

	// Preset scenes
	if (window.isPressed(GLFW_KEY_1))
		resetScene(1);
	if (window.isPressed(GLFW_KEY_2))
		resetScene(2);
	if (window.isPressed(GLFW_KEY_3))
		resetScene(3);

	// Clear scene
	if (window.isPressed(GLFW_KEY_4))
		resetScene(4);

	// Add objects
	if (window.isPressed(GLFW_KEY_Z))
		addObject(0);
	if (window.isPressed(GLFW_KEY_X))
		addObject(1);
	if (window.isPressed(GLFW_KEY_C))
		addObject(2);
}