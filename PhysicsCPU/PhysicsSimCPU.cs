using Tutorial;
using System.Numerics;
using System.Diagnostics;
using ObjLoader.Loader.Data.VertexData;
using System;
using Silk.NET.Input;

class PhysicsSimCPU
{
    private static Dictionary<int, Thread> threads = new();
    private static Dictionary<int, ManualResetEventSlim> triggers = new();
    private static CountdownEvent countdownEvent;
    private static CancellationTokenSource cancellationTokenSource = new();
    private static readonly object threadLock = new();
    private static double deltaTime = 0.01f;

    private static readonly Vector3 GRAVITY = new Vector3(0, -9.81f, 0);
    private static readonly float dampenFactor = 0.98f;
    private static readonly float BOUNDARY_MIN = -30;
    private static readonly float BOUNDARY_MAX = 30;

    private static Stopwatch stopwatch = Stopwatch.StartNew();
    public static float currentFramePhysicsMs = 0;

    private static int GetPhysicalThreadCount() => Environment.ProcessorCount;

    private static float pythagoreanSolve(float a, float b)
    {
        return (float)Math.Sqrt(a * a + b * b);
    }

    public static void applyBoundaryCollision(int index)
    {
        GameObject gameObject = Tutorial.MyProgram.gameObjects[index];
        float min = BOUNDARY_MIN + gameObject.scale;
        float max = BOUNDARY_MAX - gameObject.scale;

        if (gameObject.position.X < min)
        {
            gameObject.position.X = min;
            gameObject.velocity.X *= -1;
            gameObject.velocity *= 0.5f;
        }
        else if (gameObject.position.X > max)
        {
            gameObject.position.X = max;
            gameObject.velocity.X *= -1;
            gameObject.velocity *= 0.5f;
        }

        if (gameObject.position.Y < min)
        {
            gameObject.position.Y = min;
            gameObject.velocity.Y *= -1;
            gameObject.velocity *= 0.5f;
        }
        else if (gameObject.position.Y > max)
        {
            gameObject.position.Y = max;
            gameObject.velocity.Y *= -1;
            gameObject.velocity *= 0.5f;
        }

        if (gameObject.position.Z < min)
        {
            gameObject.position.Z = min;
            gameObject.velocity.Z *= -1;
            gameObject.velocity *= 0.5f;
        }
        else if (gameObject.position.Z > max)
        {
            gameObject.position.Z = max;
            gameObject.velocity.Z *= -1;
            gameObject.velocity *= 0.5f;
        }

        Tutorial.MyProgram.gameObjects[index] = gameObject;
    }

    public static Vector3 checkNormalization(Vector3 check)
    {
        if (float.IsNaN(check.X) || float.IsNaN(check.Y) || float.IsNaN(check.Z)
        || float.IsInfinity(check.X) || float.IsInfinity(check.Y) || float.IsInfinity(check.Z)
        || float.IsNegativeInfinity(check.X) || float.IsNegativeInfinity(check.Y) || float.IsNegativeInfinity(check.Z)
        )
        {
            return Vector3.Zero;
        }

        return check;
    }

    public static Vector3 checkCollision(int indexA, int indexB, ref float penetrationDepth)
    {
        GameObject gameObject1 = MyProgram.gameObjects[indexA];
        GameObject gameObject2 = MyProgram.gameObjects[indexB];

        Vector3 normal = Vector3.Normalize(gameObject2.position - gameObject1.position);
        normal = checkNormalization(normal);
        Vector3 correction = Vector3.Zero;

        float inverseMassA = 1.0f / gameObject1.mass;
        float inverseMassB = 1.0f / gameObject2.mass;

        const float percent = 0.2f;
        const float slop = 0.01f;

        // if box/cylinder and box/cylinder
        if ((gameObject1.meshType == MeshType.Box || gameObject1.meshType == MeshType.Cylinder) && (gameObject2.meshType == MeshType.Box || gameObject2.meshType == MeshType.Cylinder))
        {
            Vector3 n = gameObject1.position - gameObject2.position;

            Vector3 aMax = new Vector3(gameObject1.position.X + gameObject1.scale, gameObject1.position.Y + gameObject1.scale, gameObject1.position.Z + gameObject1.scale);
            Vector3 aMin = new Vector3(gameObject1.position.X - gameObject1.scale, gameObject1.position.Y - gameObject1.scale, gameObject1.position.Z - gameObject1.scale);
            Vector3 bMax = new Vector3(gameObject2.position.X + gameObject2.scale, gameObject2.position.Y + gameObject2.scale, gameObject2.position.Z + gameObject2.scale);
            Vector3 bMin = new Vector3(gameObject2.position.X - gameObject2.scale, gameObject2.position.Y - gameObject2.scale, gameObject2.position.Z +- gameObject2.scale);

            float a_extent_x = (aMax.X - aMin.X) / 2.0f;
            float b_extent_x = (bMax.X - bMin.X) / 2.0f;
            float a_extent_y = (aMax.Y - aMin.Y) / 2.0f;
            float b_extent_y = (bMax.Y - bMin.Y) / 2.0f;
            float a_extent_z = (aMax.Z - aMin.Z) / 2.0f;
            float b_extent_z = (bMax.Z - bMin.Z) / 2.0f;

            float x_overlap = a_extent_x + b_extent_x - Math.Abs(n.X);
            float y_overlap = a_extent_y + b_extent_y - Math.Abs(n.Y);
            float z_overlap = a_extent_z + b_extent_z - Math.Abs(n.Z);

            if (x_overlap > 0 && y_overlap > 0 && z_overlap > 0)
            {
                // Determine the axis of least penetration
                if (x_overlap < y_overlap && x_overlap < z_overlap)
                {
                    n = (n.X < 0) ? new Vector3(-1, 0, 0) : new Vector3(1, 0, 0);
                    penetrationDepth = x_overlap;
                }
                else if (y_overlap < z_overlap)
                {
                    n = (n.Y < 0) ? new Vector3(0, -1, 0) : new Vector3(0, 1, 0);
                    penetrationDepth = y_overlap;
                }
                else
                {
                    n = (n.Z < 0) ? new Vector3(0, 0, -1) : new Vector3(0, 0, 1);
                    penetrationDepth = z_overlap;
                }

                Vector3 midpoint = new Vector3
                (
                    (gameObject1.position.X + gameObject2.position.X) / 2.0f,
                    (gameObject1.position.Y + gameObject2.position.Y) / 2.0f,
                    (gameObject1.position.Z + gameObject2.position.Z) / 2.0f
                );

                // Positional correction only on the axis with the least penetration
                if (x_overlap < y_overlap && x_overlap < z_overlap)
                {
                    correction.X = (Math.Max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * Math.Sign(normal.X);
                }
                else if (y_overlap < z_overlap)
                {
                    correction.Y = (Math.Max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * Math.Sign(normal.Y);
                }
                else
                {
                    correction.Z = (Math.Max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * Math.Sign(normal.Z);
                }

                MyProgram.gameObjects[indexA].position -= inverseMassA * correction;
                MyProgram.gameObjects[indexB].position += inverseMassB * correction;

                return midpoint;
            }
        }
        // else sphere - sphere
        else if (gameObject1.meshType == MeshType.Sphere && gameObject2.meshType == MeshType.Sphere)
        {
            float radiusA = gameObject1.scale;
            float radiusB = gameObject2.scale;
            double distance = Math.Sqrt
            (
                (gameObject2.position.X - gameObject1.position.X) * (gameObject2.position.X - gameObject1.position.X) +
                (gameObject2.position.Y - gameObject1.position.Y) * (gameObject2.position.Y - gameObject1.position.Y) +
                (gameObject2.position.Z - gameObject1.position.Z) * (gameObject2.position.Z - gameObject1.position.Z)
            );

            if(distance <= radiusA + radiusB)
            {
                Vector3 midpoint = new Vector3
                (
                    gameObject2.position.X - gameObject1.position.X,
                    gameObject2.position.Y - gameObject1.position.Y,
                    gameObject2.position.Z - gameObject1.position.Z
                );
                midpoint *= radiusA / (radiusA + radiusB);
                penetrationDepth = radiusA + radiusB - (float)distance;

                // Positional correction
                {
                    correction = (Math.Max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * normal;

                    MyProgram.gameObjects[indexA].position -= inverseMassA * correction;
                    MyProgram.gameObjects[indexB].position += inverseMassB * correction;
                }

                return midpoint;
            }
        }
        // else sphere - box/cylinder
        else if((gameObject1.meshType == MeshType.Sphere && (gameObject2.meshType == MeshType.Box || gameObject2.meshType == MeshType.Cylinder))
            || (gameObject2.meshType == MeshType.Sphere && (gameObject1.meshType == MeshType.Box || gameObject1.meshType == MeshType.Cylinder)))
        {
            ref GameObject sphere = ref gameObject1;
            ref GameObject box = ref gameObject2;

            if(gameObject2.meshType == MeshType.Sphere)
            {
                sphere = ref gameObject2;
                box = ref gameObject1;
            }

            Vector3 halfScale = new Vector3(box.scale);
            Vector3 min = box.position - halfScale;
            Vector3 max = box.position + halfScale;

            Vector3 closestPointOnBox = new Vector3(
                Math.Clamp(sphere.position.X, min.X, max.X),
                Math.Clamp(sphere.position.Y, min.Y, max.Y),
                Math.Clamp(sphere.position.Z, min.Z, max.Z)
            );

            Vector3 difference = sphere.position - closestPointOnBox;
            float distanceSquared = difference.LengthSquared();

            // Check if there's a collision
            bool isCollision = distanceSquared <= sphere.scale * sphere.scale;

            if (isCollision)
            {
                // Calculate the penetration depth
                float distance = MathF.Sqrt(distanceSquared);
                penetrationDepth = sphere.scale - distance;

                Vector3 correctionDirection = sphere.position - closestPointOnBox;

                if (distance > 0.0f)
                {
                    correctionDirection = Vector3.Normalize(correctionDirection);
                    correctionDirection = checkNormalization(correctionDirection);

                    correction = (Math.Max(penetrationDepth - slop, 0.0f) / (inverseMassA + inverseMassB)) * percent * correctionDirection;

                    MyProgram.gameObjects[indexA].position -= inverseMassA * correction;
                    MyProgram.gameObjects[indexB].position += inverseMassB * correction;
                }

                // Return the contact point and penetration depth
                return closestPointOnBox;
            }

            // No collision
            return Vector3.Zero;
        }

        return Vector3.Zero;
    }

    public static void applyForces(int indexA, int indexB, Vector3 contactPoint, float penetrationDepth)
    {
        GameObject A = MyProgram.gameObjects[indexA];
        GameObject B = MyProgram.gameObjects[indexB];

        Vector3 normal = Vector3.Normalize(B.position - A.position);
        normal = checkNormalization(normal);
        Vector3 rA = contactPoint - A.position;
        Vector3 rB = contactPoint - B.position;
        Vector3 vA = A.velocity;
        Vector3 vB = B.velocity;

        Vector3 relativeVelocity = B.velocity - A.velocity;
        float velocityAlongNormal = Vector3.Dot(relativeVelocity, normal);

        float distance = Vector3.Distance(A.position, B.position);
        //float penetration = A.scale + B.scale - distance;
        float elasticity = 0.5f;

        float inverseMassA = 1.0f / A.mass;
        float inverseMassB = 1.0f / B.mass;

        float j = -(1.0f + elasticity) * velocityAlongNormal;
        j /= inverseMassA + inverseMassB;

        Vector3 impulse = j * normal;

        MyProgram.gameObjects[indexA].velocity -= inverseMassA * impulse;
        MyProgram.gameObjects[indexB].velocity += inverseMassB * impulse;
        
        // Friction
        {
            relativeVelocity = MyProgram.gameObjects[indexB].velocity - MyProgram.gameObjects[indexA].velocity;
            float dot = Vector3.Dot(relativeVelocity, normal);
            Vector3 n = dot * normal;
            Vector3 tangent = relativeVelocity - n;
            tangent = Vector3.Normalize(tangent);
            tangent = checkNormalization(tangent);

            float jt = -Vector3.Dot(relativeVelocity, tangent);
            jt /= inverseMassA + inverseMassB;

            float staticFriction = 0.3f;
            float dynamicFriction = 0.4f;

            float mu = pythagoreanSolve(staticFriction, staticFriction);

            Vector3 frictionImpulse = Vector3.Zero;

            if(Math.Abs(jt) < j * mu)
            {
                frictionImpulse = jt * tangent;
            }
            else
            {
                dynamicFriction = pythagoreanSolve(dynamicFriction, dynamicFriction);
                frictionImpulse = -j * tangent * dynamicFriction;
            }

            MyProgram.gameObjects[indexA].velocity -= (inverseMassA) * frictionImpulse;
            MyProgram.gameObjects[indexB].velocity += (inverseMassB) * frictionImpulse;
        }
        
    }

    public static void StepSimulation(int threadIndex)
    {
        int numberOfThreads = GetPhysicalThreadCount();
        int numberOfObjects = MyProgram.gameObjectCount;

        // Calculate the workload for this thread
        int objectsPerThread = numberOfObjects / numberOfThreads;
        int remainder = numberOfObjects % numberOfThreads;

        // Start index for this thread
        int startIndex = threadIndex * objectsPerThread + Math.Min(threadIndex, remainder);
        int stopIndex = startIndex + objectsPerThread + (threadIndex < remainder ? 1 : 0);

        for (int index = startIndex; index < stopIndex; index++)
        {
            if (index >= MyProgram.gameObjectCount)
            {
                return;
            }

            if (!MyProgram.gameObjects[index].shouldCalculate)
            {
                return;
            }

            MyProgram.gameObjects[index].velocity += GRAVITY * (float)deltaTime;

            for (int i = index + 1; i < MyProgram.gameObjectCount; i++)
            {
                float penetrationDepth = 0;
                Vector3 contact = checkCollision(index, i, ref penetrationDepth);
                if (contact != Vector3.Zero)
                {
                    applyForces(index, i, contact, penetrationDepth);
                }
            }

            applyBoundaryCollision(index);

            MyProgram.gameObjects[index].velocity *= dampenFactor;
            MyProgram.gameObjects[index].velocity /= 2;
            MyProgram.gameObjects[index].position += MyProgram.gameObjects[index].velocity;
        }
    }

    public static void ThreadLoop(int index, ManualResetEventSlim trigger, CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                trigger.Wait(token); // Wait for trigger signal
                trigger.Reset();

                int x = int.Parse(Thread.CurrentThread.Name.Substring(6));

                StepSimulation(x);
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
            }
            finally
            {
                countdownEvent.Signal(); // Signal that this thread has completed
            }
        }
    }

    public static void stepSimulationSingleThreaded(float dt)
    {
        deltaTime = dt;
        for (int i = 0; i < MyProgram.gameObjectCount; i++)
        {
            StepSimulation(i);
        }
    }

    public static void AddObject()
    {
        lock (threadLock)
        {
            int threadIndex = threads.Count;

            var trigger = new ManualResetEventSlim(false);
            triggers.Add(threadIndex, trigger);
            var thread = new Thread(() => ThreadLoop(threadIndex, trigger, cancellationTokenSource.Token));
            thread.Name = "Thread" + threadIndex;
            threads.Add(threadIndex, thread);
            thread.Start();
        }
    }

    public static void TriggerAllThreads(double dt)
    {
        float start = stopwatch.ElapsedMilliseconds;

        lock (threadLock)
        {
            deltaTime = dt;

            countdownEvent = new CountdownEvent(triggers.Count);

            foreach (var trigger in triggers)
            {
                trigger.Value.Set();
            }
        }

        countdownEvent.Wait();

        float finish = stopwatch.ElapsedMilliseconds;
        currentFramePhysicsMs = finish - start;
    }

    private static void CancelAllThreads()
    {
        cancellationTokenSource.Cancel();
        TriggerAllThreads(1);
        cancellationTokenSource = new CancellationTokenSource();
        threads.Clear();
        triggers.Clear();
    }

    public static void LoadObjects()
    {
        CancelAllThreads();

        for (int i = 0; i < GetPhysicalThreadCount(); i++)
        {
            ManualResetEventSlim trigger = new ManualResetEventSlim(false);
            triggers.Add(i, trigger);
            var thread = new Thread(() => ThreadLoop(i, trigger, cancellationTokenSource.Token));
            thread.Name = "Thread" + i;
            threads.Add(i, thread);
            thread.Start();
        }

        currentFramePhysicsMs = 0;
    }
}