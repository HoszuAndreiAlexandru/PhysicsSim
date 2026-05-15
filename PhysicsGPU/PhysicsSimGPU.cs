using ManagedCuda;
using System.Diagnostics;
using System.Numerics;
using System.Runtime.InteropServices;
using Tutorial;

[StructLayout(LayoutKind.Sequential)]
struct GameObjectData
{
    public float positionX;
    public float positionY;
    public float positionZ;
    public float scale;
    public int meshType; // We use int because CUDA doesn't directly support enums
    public int shouldCalculate;
    float velocityX;
    float velocityY;
    float velocityZ;

    // Constructor for convenience
    public GameObjectData(Vector3 position, float scale, MeshType meshType, int shouldCalculate)
    {
        this.positionX = position.X;
        this.positionY = position.Y;
        this.positionZ = position.Z;
        this.scale = scale;
        this.meshType = (int)meshType; // Store the MeshType as an int
        this.shouldCalculate = shouldCalculate; // Store the MeshType as an int
        this.velocityX = 0;
        this.velocityY = 0;
        this.velocityZ = 0;
    }
}

class PhysicsSimGPU
{
    private static int numberOfObjects = 0; // total number of data points
    private static int totalOperations = numberOfObjects * numberOfObjects; // total number of interactions
    private static int blockSize = 256; // number of threads per block
    private static int numBlocks;

    private static int numberOfAttributes = 9;
    //private static int numberOfBytes = numberOfAttributes * 4;

    private static CudaContext context;
    private static CudaKernel module;

    private static float[] flattenedData;

    private static CudaDeviceVariable<float> inputDataDevice;
    private static CudaDeviceVariable<float> output1DataDevice;

    private static Stopwatch stopwatch = Stopwatch.StartNew();
    public static float currentFramePhysicsMs = 0;

    public static void StepSimulation(int simulatedObjects, double deltaTime)
    {
        if(context == null || module == null || inputDataDevice == null || output1DataDevice == null)
        {
            return;
        }

        float start = stopwatch.ElapsedMilliseconds;

        // Create an array of parameters to pass to the kernel
        object[] parameters = new object[]
        {
                inputDataDevice.DevicePointer, // Pointer to device memory for game objects
                simulatedObjects, // Number of objects (for grid calculation)
                (float)deltaTime
        };
        module.Run(parameters);

        //DriverAPINativeMethods.

        float[] resultData = new float[flattenedData.Length];
        inputDataDevice.CopyToHost(resultData);

        //context.Synchronize();

        for (int i = 0; i < resultData.Length; i += numberOfAttributes)
        {
            MyProgram.gameObjects[i / numberOfAttributes].position.X = resultData[i];
            MyProgram.gameObjects[i / numberOfAttributes].position.Y = resultData[i + 1];
            MyProgram.gameObjects[i / numberOfAttributes].position.Z = resultData[i + 2];
        }

        float finish = stopwatch.ElapsedMilliseconds;
        currentFramePhysicsMs = finish - start;
    }

    public static float[] FlattenGameObjectData(GameObjectData[] gameObjects)
    {
        float[] flattenedData = new float[gameObjects.Length * numberOfAttributes];
        int index = 0;

        foreach (var gameObject in gameObjects)
        {
            flattenedData[index++] = gameObject.positionX;
            flattenedData[index++] = gameObject.positionY;
            flattenedData[index++] = gameObject.positionZ;
            flattenedData[index++] = gameObject.scale;
            flattenedData[index++] = gameObject.meshType;
            flattenedData[index++] = gameObject.shouldCalculate;
            flattenedData[index++] = 0;
            flattenedData[index++] = 0;
            flattenedData[index++] = 0;
        }

        return flattenedData;
    }

    public static int getBlockSize(int numberOfObjects)
    {
        int blockSize = 1;
        while(blockSize <= numberOfObjects && blockSize != 1024)
        {
            blockSize *= 2;
        }
        return blockSize;
    }

    public static void Reset()
    {
        if (inputDataDevice != null)
        {
            inputDataDevice.Dispose();
            inputDataDevice = null;
        }
        if (output1DataDevice != null)
        {
            output1DataDevice.Dispose();
            output1DataDevice = null;
        }

        currentFramePhysicsMs = 0;
    }

    public static void AddObject(GameObject go)
    {
        if (inputDataDevice == null)
        {
            numberOfObjects = MyProgram.gameObjects.Count;
            //totalOperations = numberOfObjects * numberOfObjects;
            totalOperations = MyProgram.gameObjects.Count;
            blockSize = 1024;//getBlockSize(numberOfObjects);
            numBlocks = (int)Math.Ceiling((float)totalOperations / blockSize);
            context = new CudaContext();
            module = context.LoadKernelPTX("../../../PhysicsGPU/kernel.ptx", "kernel");
            module.GridDimensions = numBlocks;
            module.BlockDimensions = blockSize;
            PhysicsSimGPU.inputDataDevice = new CudaDeviceVariable<float>(MyProgram.gameObjects.Count * numberOfAttributes);
            PhysicsSimGPU.output1DataDevice = new CudaDeviceVariable<float>(MyProgram.gameObjects.Count * numberOfAttributes);
        }

        GameObjectData[] gameObjectDataArray = MyProgram.gameObjects.Select(go => new GameObjectData(go.position, go.scale, go.meshType, go.shouldCalculate ? 1 : 0)).ToArray();
        flattenedData = FlattenGameObjectData(gameObjectDataArray);

        inputDataDevice.CopyToDevice(flattenedData);
    }

    public static void ReloadObjects()
    {
        GameObjectData[] gameObjectDataArray = MyProgram.gameObjects.Select(go => new GameObjectData(go.position, go.scale, go.meshType, go.shouldCalculate ? 1 : 0)).ToArray();
        flattenedData = FlattenGameObjectData(gameObjectDataArray);

        if (inputDataDevice != null)
        {
            inputDataDevice.CopyToDevice(flattenedData);
        }
    }

    public static void LoadObjects(List<GameObject> gameObjects)
    {
        numberOfObjects = gameObjects.Count;
        totalOperations = gameObjects.Count;
        //totalOperations = numberOfObjects * numberOfObjects;
        blockSize = 1024;//getBlockSize(numberOfObjects);
        numBlocks = (int)Math.Ceiling((float)totalOperations / blockSize);

        if(context != null)
        {
            context.Dispose();
        }

        // Create CUDA context and load kernel function
        context = new CudaContext();

        module = context.LoadKernelPTX("../../../../PhysicsGPU/kernel.ptx", "kernel");
        module.GridDimensions = numBlocks;
        module.BlockDimensions = blockSize;

        // De-allocate device memory for input and output data
        if (inputDataDevice != null)
        {
            inputDataDevice.Dispose();
        }
        if (output1DataDevice != null)
        {
            output1DataDevice.Dispose();
        }

        // Parse gameObjects into usable inputData
        GameObjectData[] gameObjectDataArray = gameObjects.Select(go => new GameObjectData(go.position, go.scale, go.meshType, go.shouldCalculate ? 1 : 0)).ToArray();
        flattenedData = FlattenGameObjectData(gameObjectDataArray);

        // allocate device memory for input and output data
        inputDataDevice = new CudaDeviceVariable<float>(gameObjects.Count * numberOfAttributes);
        output1DataDevice = new CudaDeviceVariable<float>(gameObjects.Count * numberOfAttributes);

        // Copy input data to device memory
        inputDataDevice.CopyToDevice(flattenedData);
    }
}
