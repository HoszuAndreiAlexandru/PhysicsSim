
using Silk.NET.Maths;
using System.Numerics;

public enum MeshType
{
    // Define different MeshType enum values based on your needs
    Box,
    Sphere,
    Cylinder
}

class GameObject
{
    public bool shouldCalculate = false;

    public Vector3 position = Vector3.Zero;
    public int positionIndex = 0;
    public Vector3 rotation = Vector3.Zero;
    public float scale = 1.0f;

    // Visual
    public Vector3 colour = Vector3.One;

    // Physics
    public MeshType meshType;
    public float mass = 1.0f;
    public Vector3 velocity = Vector3.Zero;
    /*
    public Vector3 acceleration = Vector3.Zero;
    public Vector3 angularVelocity = Vector3.Zero;
    public Vector3 angularAcceleration = Vector3.Zero;
    public float staticFriction = 0.4f;
    public float dynamicFriction = 0.8f;
    */

    public GameObject() {}

    public GameObject(Vector3 position, Vector3 colour)
    {
        //this.position = position;
        this.colour = colour;
    }
}