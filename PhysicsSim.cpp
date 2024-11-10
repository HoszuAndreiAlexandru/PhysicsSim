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
		float mass = 1;
		float inverseMass = 1 / mass;
		glm::vec3 rotation = glm::vec3(0);
		glm::vec3 velocity = glm::vec3(0.0f);
		glm::vec3 acceleration = glm::vec3(0.0f);
		glm::vec3 halfExtents = glm::vec3(1.0f);
		glm::vec3 angularVelocity = glm::vec3(0.0f);
		glm::vec3 angularAcceleration = glm::vec3(0.0f);
		double staticFriction = 0.4f;
		double dynamicFriction = 0.8f;

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
const glm::vec3 GRAVITY(0.0f, -9.81f, 0.0f); // Gravity vector

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

double pythagoreanSolve(double a, double b)
{
	return glm::sqrt(a * a + b * b);
}

void applyForces(GameObject& gameObject1, GameObject& gameObject2, float dampenFactor = 0.1f)
{
	// Calculate radii from COM to contact
	glm::vec3 ra = contacts[i] - A->position;
	glm::vec3 rb = contacts[i] - B->position;

	// Relative velocity
	glm::vec3 rv = B->velocity + Cross(B->angularVelocity, rb) -
		A->velocity - Cross(A->angularVelocity, ra);

	// Relative velocity along the normal
	double contactVel = glm::dot(rv, normal);

	// Do not resolve if velocities are separating
	if (contactVel > 0)
		return;

	real raCrossN = Cross(ra, normal);
	real rbCrossN = Cross(rb, normal);
	real invMassSum = A->im + B->im + Sqr(raCrossN) * A->iI + Sqr(rbCrossN) * B->iI;

	// Calculate impulse scalar
	real j = -(1.0f + e) * contactVel;
	j /= invMassSum;
	j /= (real)contact_count;

	// Apply impulse
	Vec2 impulse = normal * j;
	A->ApplyImpulse(-impulse, ra);
	B->ApplyImpulse(impulse, rb);

	// Friction impulse
	rv = B->velocity + Cross(B->angularVelocity, rb) -
		A->velocity - Cross(A->angularVelocity, ra);

	Vec2 t = rv - (normal * Dot(rv, normal));
	t.Normalize();

	// j tangent magnitude
	real jt = -Dot(rv, t);
	jt /= invMassSum;
	jt /= (real)contact_count;

	// Don't apply tiny friction impulses
	if (Equal(jt, 0.0f))
		return;

	// Coulumb's law
	Vec2 tangentImpulse;
	if (std::abs(jt) < j * sf)
		tangentImpulse = t * jt;
	else
		tangentImpulse = t * -j * df;

	// Apply friction impulse
	A->ApplyImpulse(-tangentImpulse, ra);
	B->ApplyImpulse(tangentImpulse, rb);

	/*
	// Calculate the collision normal (direction from objA to objB)
	glm::vec3 collisionNormal = glm::normalize(gameObject2.position - gameObject1.position);

	// Calculate relative velocity along the collision normal
	glm::vec3 relativeVelocity = gameObject2.velocity - gameObject1.velocity;
	float velocityAlongNormal = glm::dot(relativeVelocity, collisionNormal);

	// If the objects are moving apart, no need to apply forces
	if (velocityAlongNormal > 0) return;

	// Calculate restitution (bounciness) using the dampenFactor
	float restitution = dampenFactor;

	// Calculate the impulse scalar
	float impulseScalar = -(1 + restitution) * velocityAlongNormal;

	// Calculate the impulse vector
	glm::vec3 impulse = impulseScalar * collisionNormal;

	// Apply the impulse to each object's velocity (linear response)
	gameObject1.velocity -= impulse;
	gameObject2.velocity += impulse;

	// ---- Step 1: Calculate Tangential (Friction) Impulse ----

	// Find the tangent vector for the friction impulse
	
	glm::vec3 tangent = relativeVelocity - glm::dot(relativeVelocity, collisionNormal) * collisionNormal;
	tangent = glm::normalize(tangent);

	// Calculate the magnitude of the friction impulse
	float frictionCoefficient = 0.1f; // adjust this value for more or less friction
	float frictionImpulseMagnitude = -glm::dot(relativeVelocity, tangent) * frictionCoefficient;
	frictionImpulseMagnitude /= (gameObject1.inverseMass + gameObject2.inverseMass);

	float mu = pythagoreanSolve(gameObject1.staticFriction, gameObject2.staticFriction);

	// Apply friction impulse in the direction of the tangent
	glm::vec3 frictionImpulse = frictionImpulseMagnitude * tangent;

	if (abs(frictionImpulseMagnitude) >= impulseScalar * mu)
	{
		float dynamicFriction = pythagoreanSolve(gameObject1.dynamicFriction, gameObject2.dynamicFriction);
		frictionImpulse = -impulseScalar * tangent * dynamicFriction;
	}

	gameObject1.velocity -= glm::vec3(gameObject1.inverseMass) * frictionImpulse;
	gameObject2.velocity += glm::vec3(gameObject2.inverseMass) * frictionImpulse;

	//float penX = (gameObject2.halfExtents.x + gameObject1.halfExtents.x) - std::abs(gameObject2.position.x - gameObject1.position.x);
	//float penY = (gameObject2.halfExtents.y + gameObject1.halfExtents.y) - std::abs(gameObject2.position.y - gameObject1.position.y);
	//float penZ = (gameObject2.halfExtents.z + gameObject1.halfExtents.z) - std::abs(gameObject2.position.z - gameObject1.position.z);
	//glm::vec3 penetrationDepth = glm::vec3(penX, penY, penZ);

	

	//const float percent = 0.2; // usually 20% to 80%
	//const float slop = 0.1;
	//glm::vec3 correction = glm::max(penetrationDepth - slop, 0.0f) / (gameObject1.inverseMass + gameObject2.inverseMass) * percent * collisionNormal;
	//gameObject1.position -= glm::vec3(gameObject1.inverseMass) * correction;
	//gameObject2.position += glm::vec3(gameObject2.inverseMass) * correction;




	// ---- Step 2: Apply Angular Impulse for Rotation ----

	// Assume center of mass at position and calculate collision points

	
	
	glm::vec3 collisionPoint1 = gameObject1.position + collisionNormal * glm::vec3(gameObject1.scale);
	glm::vec3 collisionPoint2 = gameObject2.position - collisionNormal * glm::vec3(gameObject2.scale);

	// Calculate relative position vectors from centers of mass to collision points
	glm::vec3 r1 = collisionPoint1 - gameObject1.position;
	glm::vec3 r2 = collisionPoint2 - gameObject2.position;

	// Calculate torque induced by the impulse
	glm::vec3 torque1 = glm::cross(r1, impulse + frictionImpulse);
	glm::vec3 torque2 = glm::cross(r2, -(impulse + frictionImpulse));

	// Update angular velocity (assuming a unit moment of inertia for simplicity)
	float momentOfInertia = 1.0f; // adjust as necessary for different objects
	gameObject1.angularVelocity += torque1 / momentOfInertia;
	gameObject2.angularVelocity += torque2 / momentOfInertia;
	


	

	// Calculate the point of collision on each object relative to its center
	glm::vec3 collisionPointA = gameObject1.position + collisionNormal * glm::vec3(gameObject1.scale);
	glm::vec3 collisionPointB = gameObject2.position - collisionNormal * glm::vec3(gameObject2.scale);

	// Calculate the torque generated on each object due to off-center collision
	glm::vec3 rA = collisionPointA - gameObject1.position; // Vector from objA's center to collision point
	glm::vec3 rB = collisionPointB - gameObject2.position; // Vector from objB's center to collision point

	glm::vec3 torqueA = glm::cross(rA, impulse); // Torque on objA
	glm::vec3 torqueB = glm::cross(rB, -impulse); // Torque on objB

	// Update angular velocities (assuming a simple mass of 1 for rotation)
	gameObject1.angularVelocity += torqueA;
	gameObject2.angularVelocity += torqueB;
	*/
}

bool checkCollision(GameObject& gameObject1, GameObject& gameObject2)
{
	if ((gameObject1.meshType == BOX && gameObject2.meshType == BOX) || (gameObject1.meshType == CYLINDER && gameObject2.meshType == CYLINDER))
	{
		bool overlapX = std::abs(gameObject1.position.x - gameObject2.position.x) <= (gameObject1.halfExtents.x + gameObject2.halfExtents.x);
		bool overlapY = std::abs(gameObject1.position.y - gameObject2.position.y) <= (gameObject1.halfExtents.y + gameObject2.halfExtents.y);
		bool overlapZ = std::abs(gameObject1.position.z - gameObject2.position.z) <= (gameObject1.halfExtents.z + gameObject2.halfExtents.z);
		return overlapX && overlapY && overlapZ;
	}
	else if (gameObject1.meshType == SPHERE && gameObject2.meshType == SPHERE)
	{
		float radiusA = gameObject1.halfExtents.x;
		float radiusB = gameObject2.halfExtents.x;
		float distance = glm::distance(gameObject1.position, gameObject2.position);
		return distance <= (radiusA + radiusB);
	}
	else if ((gameObject1.meshType == SPHERE && gameObject2.meshType == BOX) || (gameObject1.meshType == BOX && gameObject2.meshType == SPHERE))
	{
		GameObject& sphere = gameObject1;
		GameObject& box = gameObject2;
		if (gameObject2.meshType == SPHERE)
		{
			sphere = gameObject2;
			box = gameObject1;
		}
		float radius = sphere.halfExtents.x;
		glm::vec3 closestPoint;
		for (int i = 0; i < 3; ++i) {
			closestPoint[i] = std::max(box.position[i] - box.halfExtents[i],
				std::min(sphere.position[i], box.position[i] + box.halfExtents[i]));
		}
		float distanceSquared = glm::distance(closestPoint, sphere.position);
		return distanceSquared <= (radius * radius);
	}

	return false;
}

void simulatePhysics()
{
	float dampenFactor = 0.2f;
	for (int i = 0; i < objects.size(); i++)
	{
		GameObject& obj = objects[i];

		obj.velocity += GRAVITY * deltaTime;

		for (int j = i + 1; j < objects.size(); j++)
		{
			if (checkCollision(objects[i], objects[j]))
			{
				applyForces(objects[i], objects[j]);
			}
		}



		/*
		// Apply gravity to acceleration
		obj.acceleration += GRAVITY;

		// Integrate acceleration to update velocity and position
		obj.velocity += obj.acceleration * deltaTime;
		obj.position += obj.velocity * deltaTime;

		// Optional friction to stabilize stacking
		float friction = 0.5f;
		obj.velocity.x *= friction;
		obj.velocity.z *= friction;

		// Integrate angular velocity to update rotation
		obj.rotation += obj.angularVelocity * deltaTime;

		// Dampen angular velocity slightly for stability
		if (obj.angularVelocity.x > 0.1f || obj.angularVelocity.y > 0.1f || obj.angularVelocity.z > 0.1f || obj.angularVelocity.x < -0.1f || obj.angularVelocity.y < -0.1f || obj.angularVelocity.z < -0.1f)
		{
			//std::cout << obj.angularVelocity.x << " " << obj.angularVelocity.y << " " << obj.angularVelocity.z << std::endl;
		}
		obj.angularVelocity *= 0.98f;

		if (obj.position.x - obj.scale < BOUNDARY_MIN.x) {
			obj.position.x = BOUNDARY_MIN.x + obj.scale; // Keep object inside
			obj.velocity.x = -obj.velocity.x * dampenFactor; // Reverse and dampen velocity
		}
		else if (obj.position.x + obj.scale > BOUNDARY_MAX.x) {
			obj.position.x = BOUNDARY_MAX.x - obj.scale;
			obj.velocity.x = -obj.velocity.x * dampenFactor;
		}

		if (obj.position.y - obj.scale < BOUNDARY_MIN.y) {
			obj.position.y = BOUNDARY_MIN.y + obj.scale;
			obj.velocity.y = -obj.velocity.y * dampenFactor;
		}
		else if (obj.position.y + obj.scale > BOUNDARY_MAX.y) {
			obj.position.y = BOUNDARY_MAX.y - obj.scale;
			obj.velocity.y = -obj.velocity.y * dampenFactor;
		}

		if (obj.position.z - obj.scale < BOUNDARY_MIN.z) {
			obj.position.z = BOUNDARY_MIN.z + obj.scale;
			obj.velocity.z = -obj.velocity.z * dampenFactor;
		}
		else if (obj.position.z + obj.scale > BOUNDARY_MAX.z) {
			obj.position.z = BOUNDARY_MAX.z - obj.scale;
			obj.velocity.z = -obj.velocity.z * dampenFactor;
		}

		// Reset acceleration for the next frame
		obj.acceleration = glm::vec3(0.0f);

		if (obj.rotation.x > 0.1f || obj.rotation.y > 0.1f || obj.rotation.z > 0.1f || obj.rotation.x < -0.1f || obj.rotation.y < -0.1f || obj.rotation.z < -0.1f)
		{
			//std::cout << obj.rotation.x << " " << obj.rotation.y << " " << obj.rotation.z << std::endl;
		}

		for (int j = i + 1; j < objects.size(); j++)
		{
			if (checkCollision(objects[i], objects[j]))
			{
				applyForces(objects[i], objects[j]);
			}
		}

		*/
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

void createObject(MeshType meshType, glm::vec3 position)
{
	float distance = 30;
	GameObject go = GameObject();
	go.position = position;
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
		// test

		break;
	case 5:
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

	if (sceneId == 4)
	{
		createObject(BOX, glm::vec3(0, 0, 0));

		createObject(BOX, glm::vec3(1.5f, 3.0f, 0.0f));
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
				ModelMatrix = glm::translate(ModelMatrix, objects[i].position);
				ModelMatrix = glm::rotate(ModelMatrix, objects[i].rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
				ModelMatrix = glm::rotate(ModelMatrix, objects[i].rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));
				ModelMatrix = glm::rotate(ModelMatrix, objects[i].rotation.z, glm::vec3(0.0f, 0.0f, 1.0f));
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

	// Test scene
	if (window.isPressed(GLFW_KEY_4))
		resetScene(4);

	// Clear scene
	if (window.isPressed(GLFW_KEY_5))
		resetScene(5);

	// Add objects
	if (window.isPressed(GLFW_KEY_Z))
		addObject(0);
	if (window.isPressed(GLFW_KEY_X))
		addObject(1);
	if (window.isPressed(GLFW_KEY_C))
		addObject(2);
}