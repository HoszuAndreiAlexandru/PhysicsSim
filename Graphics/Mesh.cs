using Silk.NET.OpenGL;
using System.Numerics;
using Tutorial;

public class Mesh : IDisposable
{
    public Mesh(GL gl, float[] vertices, uint[] indices, List<Tutorial.Texture> textures, MeshType meshType)
    {
        GL = gl;
        Vertices = vertices;
        Indices = indices;
        Textures = textures;
        SetupMesh(meshType);
    }

    public float[] Vertices { get; private set; }
    public uint[] Indices { get; private set; }
    public IReadOnlyList<Tutorial.Texture>? Textures { get; private set; }
    public VertexArrayObject<float, uint>? VAO { get; set; }
    public BufferObject<float>? VBO { get; set; }
    public BufferObject<uint>? EBO { get; set; }
    public uint instanceBuffer;
    public GL GL { get; }

    public unsafe void SetupMesh(MeshType meshType)
    {
        EBO = new BufferObject<uint>(GL, Indices, BufferTargetARB.ElementArrayBuffer);
        VBO = new BufferObject<float>(GL, Vertices, BufferTargetARB.ArrayBuffer);
        VAO = new VertexArrayObject<float, uint>(GL, VBO, EBO);

        VAO.VertexAttributePointer(0, 3, VertexAttribPointerType.Float, 5, 0);
        VAO.VertexAttributePointer(1, 2, VertexAttribPointerType.Float, 5, 3);

        instanceBuffer = GL.GenBuffer();
        GL.BindBuffer(BufferTargetARB.ArrayBuffer, instanceBuffer);

        switch (meshType)
        {
            case MeshType.Box:
                fixed (Vector3* boxOffsetsPtr = MyProgram.boxOffsets)
                {
                    GL.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(MyProgram.boxOffsets.Length * sizeof(float) * 3), boxOffsetsPtr, BufferUsageARB.StaticDraw);
                }
                break;
            case MeshType.Sphere:
                fixed (Vector3* sphereOffsetsPtr = MyProgram.sphereOffsets)
                {
                    GL.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(MyProgram.sphereOffsets.Length * sizeof(float) * 3), sphereOffsetsPtr, BufferUsageARB.StaticDraw);
                }
                break;
            case MeshType.Cylinder:
                fixed (Vector3* cylinderOffsetsPtr = MyProgram.cylinderOffsets)
                {
                    GL.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(MyProgram.cylinderOffsets.Length * sizeof(float) * 3), cylinderOffsetsPtr, BufferUsageARB.StaticDraw);
                }
                break;
            default:
                break;
        }
        /*

        GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 8 * sizeof(float), 0);
        GL.EnableVertexAttribArray(2);
        GL.VertexAttribDivisor(2, 1);
        */
    }

    public void Bind()
    {
        VAO!.Bind();
    }

    public void Dispose()
    {
        Textures = null;
        VAO!.Dispose();
        VBO!.Dispose();
        EBO!.Dispose();
    }
}