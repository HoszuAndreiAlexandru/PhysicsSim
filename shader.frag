#version 330 core
in vec2 fUv;

uniform vec3 objectColor;

out vec4 FragColor;

void main()
{
    FragColor = vec4(objectColor, 1.0f);
}