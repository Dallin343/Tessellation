#version 410

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
layout (location=2) in vec3 _vertexColor;
layout (location=3) in vec2 texCoords;

out vec4 vertexColor;
out vec2 TexCoord;
out vec3 vertexNormal;

void main() {
    gl_Position = vec4(position, 1.0);
    vertexColor = vec4(_vertexColor, 1.0);
    TexCoord = texCoords;
    vertexNormal = normal;
}
