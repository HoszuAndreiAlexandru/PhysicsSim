using System.Numerics;

public class Camera
{
    public Vector3 Position { get; set; }
    public Vector3 ViewDirection { get; set; }
    public Vector3 Up { get; set; }
    public Vector3 Right { get; set; }

    public float RotationOx { get; set; }
    public float RotationOy { get; set; }

    public Camera()
    {
        Position = new Vector3(0.0f, 0.0f, -65.0f);
        ViewDirection = new Vector3(0.0f, 0.0f, -1.0f);
        Up = new Vector3(0.0f, 1.0f, 0.0f);
        RotationOx = -90.0f;
        RotationOy = 0.0f;
        Right = Vector3.Cross(ViewDirection, Up);
    }

    public Camera(Vector3 cameraPosition)
    {
        Position = cameraPosition;
        ViewDirection = new Vector3(0.0f, 0.0f, -1.0f);
        Up = new Vector3(0.0f, 1.0f, 0.0f);
        RotationOx = 0.0f;
        RotationOy = -90.0f;
        Right = Vector3.Cross(ViewDirection, Up);
    }

    public void KeyboardMoveFront(float cameraSpeed)
    {
        Position += ViewDirection * cameraSpeed;
    }

    public void KeyboardMoveBack(float cameraSpeed)
    {
        Position -= ViewDirection * cameraSpeed;
    }

    public void KeyboardMoveLeft(float cameraSpeed)
    {
        Position -= Vector3.Normalize(Vector3.Cross(ViewDirection, Up)) * cameraSpeed;
    }

    public void KeyboardMoveRight(float cameraSpeed)
    {
        Position += Vector3.Normalize(Vector3.Cross(ViewDirection, Up)) * cameraSpeed;
    }

    public void KeyboardMoveUp(float cameraSpeed)
    {
        Position += Up * cameraSpeed;
    }

    public void KeyboardMoveDown(float cameraSpeed)
    {
        Position -= Up * cameraSpeed;
    }

    /*
    public void RotateOx(float angle)
    {
        Vector3 cameraRight = Vector3.Cross(ViewDirection, Up);
        ViewDirection = Vector3.Normalize(new Vector3(Matrix3.CreateFromAxisAngle(cameraRight, angle) * new Vector4(ViewDirection, 1)));
        Up = Vector3.Normalize(Vector3.Cross(cameraRight, ViewDirection));
    }

    public void RotateOy(float angle)
    {
        Vector3 cameraRight = Vector3.Cross(ViewDirection, Up);
        ViewDirection = Vector3.Normalize(new Vector3(Matrix4.CreateFromAxisAngle(Up, angle) * new Vector4(ViewDirection, 1)));

        cameraRight = Vector3.Normalize(new Vector3(Matrix4x4.CreateFromAxisAngle(Up, angle) * new Vector4(cameraRight, 1)));
        Up = Vector3.Normalize(Vector3.Cross(cameraRight, ViewDirection));
    }

    public void MouseMovement(float xpos, float ypos, int windowWidth, int windowHeight)
    {
        float sensitivity = 0.01f;

        float xoffset = -(xpos - windowWidth / 2) * sensitivity;
        float yoffset = (windowHeight / 2 - ypos) * sensitivity;

        RotationOy += xoffset;
        RotationOx -= yoffset;

        Update(xoffset, yoffset);
    }

    public void Update(float xoffset, float yoffset)
    {
        Vector3 newViewDirection;
        newViewDirection.X = (float)Math.Cos(RotationOy) * (float)Math.Sin(RotationOx);
        newViewDirection.Y = (float)Math.Sin(RotationOy);
        newViewDirection.Z = (float)Math.Cos(RotationOy) * (float)Math.Cos(RotationOx);

        ViewDirection = Matrix3x3.CreateRotationY(yoffset) * Matrix3x3.CreateRotationX(xoffset) * ViewDirection;
        Right = Vector3.Normalize(Vector3.Cross(ViewDirection, Up));
    }
    */
}