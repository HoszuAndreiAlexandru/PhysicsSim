#include <cuda_runtime.h>

// Define the GameObjectData structure in C++/CUDA
struct GameObjectData
{
    float positionX;
    float positionY;
    float positionZ;

    float scale;
    int meshType;
    int shouldCalculate;

    float velocityX;
    float velocityY;
    float velocityZ;
};

struct Vector3
{
    float x;
    float y;
    float z;
};

__device__ void multiply(Vector3& x, const float& y) 
{
    x.x *= y; 
    x.y *= y; 
    x.z *= y;
}

__device__ Vector3 multiply2(Vector3& x, const float& y) 
{
    return Vector3{
        x.x * y,
        x.y * y,
        x.z * y
    };
}

__device__ Vector3 add(const Vector3& x, const Vector3& y)
{
    return Vector3 {
        x.x + y.x,
        x.y + y.y,
        x.z + y.z
    };
}

__device__ Vector3 substract(const Vector3& x, const Vector3& y)
{
    return Vector3 {
        x.x - y.x,
        x.y - y.y,
        x.z - y.z
    };
}

__device__ Vector3 getPositionVector(const float* gameObjects, const int& i)
{
    return Vector3{
        gameObjects[i],
        gameObjects[i + 1],
        gameObjects[i + 2]
    };
}

__device__ Vector3 getVelocityVector(const float* gameObjects, const int& i)
{
    return Vector3{
        gameObjects[i + 6],
        gameObjects[i + 7],
        gameObjects[i + 8]
    };
}

__device__ float dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ Vector3 normalize(const Vector3& v) {
    float magnitude = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    return magnitude > 0 ? Vector3{v.x / magnitude, v.y / magnitude, v.z / magnitude} : Vector3{0.0f, 0.0f, 0.0f};
}

__device__ float Distance(const Vector3& a, const Vector3& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

__device__ float pythagoreanSolve(float a, float b)
{
    return sqrt(a * a + b * b);
}

__device__ float clamp(float value, float min, float max)
{
    if (min > max)
    {
        return 0;
    }

    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }

    return value;
}

__device__ int sign(float value)
{
    if (value < 0)
    {
        return -1;
    }
    else if (value > 0)
    {
        return 1;
    }

    return 0;
}

__device__ void multiplyVelocity(float* gameObjects, long i, float value)
{
    gameObjects[i + 6] *= value;
    gameObjects[i + 7] *= value;
    gameObjects[i + 8] *= value;
}

__device__ void applyBoundaryCollision(float* gameObjects, long i)
{
    const int min = -30 + gameObjects[i + 3];
    const int max = 30 - gameObjects[i + 3];

    if(gameObjects[i] < min)
    {
        gameObjects[i] = min;
        gameObjects[i + 6] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    }  
    else if (gameObjects[i] > max)
    {
        gameObjects[i] = max;
        gameObjects[i + 6] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    } 

    if(gameObjects[i + 1] < min)
    {
        gameObjects[i + 1] = min;
        gameObjects[i + 7] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    }  
    else if (gameObjects[i + 1] > max)
    {
        gameObjects[i + 1] = max;
        gameObjects[i + 7] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    }

    if(gameObjects[i + 2] < min)
    {
        gameObjects[i + 2] = min;
        gameObjects[i + 8] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    }  
    else if (gameObjects[i + 2] > max)
    {
        gameObjects[i + 2] = max;
        gameObjects[i + 8] *= -1;
        multiplyVelocity(gameObjects, i, 0.5f);
    }
}

__device__ Vector3 checkCollision(float* gameObjects, long i, long j, float &penetrationDepth)
{
    const float percent = 0.2f; 
    const float slop = 0.01f;

    float inverseMassA = 1.0f;
    float inverseMassB = 1.0f;

    Vector3 n = substract(getPositionVector(gameObjects, j), getPositionVector(gameObjects, i));

    // if box/cylinder and box/cylinder
    if((gameObjects[i + 4] == 0 || gameObjects[i + 4] == 2) && (gameObjects[j + 4] == 0 || gameObjects[j + 4] == 2))
    {
        float scale1 = gameObjects[i + 3];
        float scale2 = gameObjects[j + 3];

        Vector3 aMax = Vector3{gameObjects[i] + scale1, gameObjects[i + 1] + scale1, gameObjects[i + 2] + scale1};
        Vector3 aMin = Vector3{gameObjects[i] - scale1, gameObjects[i + 1] - scale1, gameObjects[i + 2] - scale1};

        Vector3 bMax = Vector3{gameObjects[j] + scale2, gameObjects[j + 1] + scale2, gameObjects[j + 2] + scale2};
        Vector3 bMin = Vector3{gameObjects[j] - scale2, gameObjects[j + 1] - scale2, gameObjects[j + 2] - scale2};

        float a_extent_x = (aMax.x - aMin.x) / 2.0f;
        float b_extent_x = (bMax.x - bMin.x) / 2.0f;
        float a_extent_y = (aMax.y - aMin.y) / 2.0f;
        float b_extent_y = (bMax.y - bMin.y) / 2.0f;
        float a_extent_z = (aMax.z - aMin.z) / 2.0f;
        float b_extent_z = (bMax.z - bMin.z) / 2.0f;

        float x_overlap = a_extent_x + b_extent_x - abs(n.x);
        float y_overlap = a_extent_y + b_extent_y - abs(n.y);
        float z_overlap = a_extent_z + b_extent_z - abs(n.z);

        if (x_overlap > 0 && y_overlap > 0 && z_overlap > 0)
        {
            // Determine the axis of least penetration
            if (x_overlap < y_overlap && x_overlap < z_overlap)
            {
                n = (n.x < 0) ? Vector3{-1, 0, 0} : Vector3{1, 0, 0};
                penetrationDepth = x_overlap;
            }
            else if (y_overlap < z_overlap)
            {
                n = (n.y < 0) ? Vector3{0, -1, 0} : Vector3{0, 1, 0};
                penetrationDepth = y_overlap;
            }
            else
            {
                n = (n.z < 0) ? Vector3{0, 0, -1} : Vector3{0, 0, 1};
                penetrationDepth = z_overlap;
            }

            Vector3 midpoint = Vector3
            {
                (gameObjects[i] + gameObjects[j] ) / 2.0f,
                (gameObjects[i + 1] + gameObjects[j + 1]) / 2.0f,
                (gameObjects[i + 2] + gameObjects[j + 2]) / 2.0f
            };
            
            // Positional correction only on the axis with the least penetration
            {
                Vector3 correction = Vector3 {0,0,0};
                if (x_overlap < y_overlap && x_overlap < z_overlap)
                {
                    correction.x = (max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * sign(n.x);
                }
                else if (y_overlap < z_overlap)
                {
                    correction.y = (max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * sign(n.y);
                }
                else
                {
                    correction.z = (max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * sign(n.z);
                }

                Vector3 correctionA = multiply2(correction, inverseMassA);
                Vector3 correctionB = multiply2(correction, inverseMassB);

                gameObjects[i] -= correctionA.x;
                gameObjects[i + 1] -= correctionA.y;
                gameObjects[i + 2] -= correctionA.z;

                gameObjects[j] += correctionB.x;
                gameObjects[j + 1] += correctionB.y;
                gameObjects[j + 2] += correctionB.z;
            }

            return midpoint;
        }
    }
    // else sphere - sphere
    else if(gameObjects[i + 4] == 1 && gameObjects[j + 4] == 1)
    {
        float radiusA = gameObjects[i + 3];
        float radiusB = gameObjects[j + 3];
        float distance = sqrt(
            (gameObjects[j] - gameObjects[i]) * (gameObjects[j] - gameObjects[i]) + 
            (gameObjects[j + 1] - gameObjects[i + 1]) * (gameObjects[j + 1] - gameObjects[i + 1]) +
            (gameObjects[j + 2] - gameObjects[i + 2]) * (gameObjects[j + 2] - gameObjects[i + 2])
        );

        if(distance <= radiusA + radiusB)
        {
            Vector3 midpoint = Vector3{
                gameObjects[j] - gameObjects[i],
                gameObjects[j + 1] - gameObjects[i + 1],
                gameObjects[j + 2] - gameObjects[i + 2],
            };
            multiply(midpoint, (radiusA / (radiusA + radiusB)));
            penetrationDepth = radiusA + radiusB - distance;

            // Positional correction
            {
                const float percent = 0.2f; 
                const float slop = 0.01f;

                Vector3 correction = multiply2(n, (max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent);
                Vector3 correctionA = multiply2(correction, inverseMassA);
                Vector3 correctionB = multiply2(correction, inverseMassB);

                gameObjects[i] -= correctionA.x;
                gameObjects[i + 1] -= correctionA.y;
                gameObjects[i + 2] -= correctionA.z;

                gameObjects[j] += correctionB.x;
                gameObjects[j + 1] += correctionB.y;
                gameObjects[j+ 2] += correctionB.z;
            }

            return midpoint;
        }
    }
    // else sphere - box/cylinder
    else
    {
        int sphere = i;
        int box = j;

        if(gameObjects[j + 4] == 1)
        {
            sphere = j;
            box = i;
        }

        Vector3 boxPosition = getPositionVector(gameObjects, box);
        Vector3 spherePosition = getPositionVector(gameObjects, sphere);

        Vector3 halfScale = Vector3{gameObjects[j + 3]};
        Vector3 min = substract(boxPosition, halfScale);
        Vector3 maxT = add(boxPosition, halfScale);

        Vector3 closestPointOnBox = Vector3{
            clamp(spherePosition.x, min.x, maxT.x),
            clamp(spherePosition.y, min.y, maxT.y),
            clamp(spherePosition.z, min.z, maxT.z)
        };

        Vector3 difference = substract(spherePosition, closestPointOnBox);
        float distanceSquared = dot(difference, difference);

        float sphereRadius = gameObjects[j + 3];

        // Check if there's a collision
        bool isCollision = distanceSquared <= sphereRadius * sphereRadius;

        if (isCollision)
        {
            // Calculate the penetration depth
            float distance = sqrt(distanceSquared);
            penetrationDepth = sphereRadius - distance;

            // Positional correction
            if (distance > 0.0f)
            {
                Vector3 correctionDirection = normalize(correctionDirection);
                Vector3 correction = multiply2(correctionDirection, (max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent);

                Vector3 correctionA = multiply2(correction, inverseMassA);
                Vector3 correctionB = multiply2(correction, inverseMassB);

                gameObjects[i] -= correctionA.x;
                gameObjects[i + 1] -= correctionA.y;
                gameObjects[i + 2] -= correctionA.z;

                gameObjects[j] += correctionB.x;
                gameObjects[j + 1] += correctionB.y;
                gameObjects[j+ 2] += correctionB.z;
            }

            // Return the contact point and penetration depth
            return closestPointOnBox;
        }

        // No collision
        return Vector3 {0,0,0};
    }

    return Vector3 {0,0,0};
}

__device__ void applyForces(float* gameObjects, long obj1, long obj2, const Vector3& contactPoint, float &penetrationDepth)
{
    Vector3 positionA = getPositionVector(gameObjects, obj1);
    Vector3 positionB = getPositionVector(gameObjects, obj2);
    Vector3 normal = normalize(substract(positionB, positionA));
    Vector3 rA = substract(contactPoint, positionA);
    Vector3 rB = substract(contactPoint, positionB);
    Vector3 vA = getVelocityVector(gameObjects, obj1);
    Vector3 vB = getVelocityVector(gameObjects, obj2);

    Vector3 relativeVelocity = substract(vB, vA);
    float velocityAlongNormal = dot(relativeVelocity, normal);

    float distance = Distance(positionA, positionB);
    float elasticity = 0.5f;

    float inverseMassA = 1.0f;
    float inverseMassB = 1.0f;

    float j = -(1.0f + elasticity) * velocityAlongNormal;
    j /= inverseMassA + inverseMassB;

    Vector3 impulse = multiply2(normal, j);
    Vector3 impulseA = multiply2(impulse, inverseMassA);
    Vector3 impulseB = multiply2(impulse, inverseMassB);

    gameObjects[obj1 + 6] -= impulseA.x;
    gameObjects[obj1 + 7] -= impulseA.y;
    gameObjects[obj1 + 8] -= impulseA.z;

    gameObjects[obj2 + 6] += impulseB.x;
    gameObjects[obj2 + 7] += impulseB.y;
    gameObjects[obj2 + 8] += impulseB.z;
    
    // Friction
    {
        vA = getVelocityVector(gameObjects, obj1);
        vB = getVelocityVector(gameObjects, obj2);
        relativeVelocity = substract(vB, vA);
        
        float dotP = dot(relativeVelocity, normal);
        Vector3 n = multiply2(normal, dotP);
        Vector3 tangent = substract(relativeVelocity, n);
        tangent = normalize(tangent);

        float jt = -dot(relativeVelocity, tangent);
        jt /= inverseMassA + inverseMassB;

        float staticFriction = 0.3f;
        float dynamicFriction = 0.4f;

        float mu = pythagoreanSolve(staticFriction, staticFriction);

        Vector3 frictionImpulse = Vector3{0,0,0};

        if(abs(jt) < j * mu)
        {
            frictionImpulse = multiply2(tangent, jt);
        }
        else
        {
            dynamicFriction = pythagoreanSolve(dynamicFriction, dynamicFriction);
            Vector3 temp = multiply2(tangent, -j);
            frictionImpulse = multiply2(temp, dynamicFriction);
        }

        Vector3 impulseA = multiply2(frictionImpulse, inverseMassA);
        Vector3 impulseB = multiply2(frictionImpulse, inverseMassB);

        gameObjects[obj1 + 6] -= impulseA.x;
        gameObjects[obj1 + 7] -= impulseA.y;
        gameObjects[obj1 + 8] -= impulseA.z;

        gameObjects[obj2 + 6] += impulseB.x;
        gameObjects[obj2 + 7] += impulseB.y;
        gameObjects[obj2 + 8] += impulseB.z;
    }
    
}

// Kernel definition using __global__ for execution on the device
extern "C" __global__ void kernel(float* gameObjects, int numObjects, float deltaTime)
{
    long index = blockIdx.x * blockDim.x + threadIdx.x;;

    if(index >= numObjects)
    {
        return;
    }

    const int numberOfAttributes = 9;
    long i = index * numberOfAttributes;

    if(gameObjects[i + 5] == 0)
    {
        return;
    }

    const float dampenFactor = 0.98f;

    
    gameObjects[i + 1] += -9.81f * deltaTime;

    for(long j = i + numberOfAttributes; j <= numObjects * numberOfAttributes; j += numberOfAttributes)
    {
        float penetrationDepth = 0;
        Vector3 contact = checkCollision(gameObjects, i, j, penetrationDepth);
        if(contact.x != 0 && contact.y != 0 && contact.z != 0)
        {
            applyForces(gameObjects, i, j, contact, penetrationDepth);
        }
    }

    gameObjects[i + 6] *= dampenFactor;
    gameObjects[i + 7] *= dampenFactor;
    gameObjects[i + 8] *= dampenFactor;

    gameObjects[i ] += gameObjects[i + 6];
    gameObjects[i + 1] += gameObjects[i + 7];
    gameObjects[i + 2] += gameObjects[i + 8];

    applyBoundaryCollision(gameObjects, i);
}