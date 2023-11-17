#version 410

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
layout (location=2) in vec3 _vertexColor;
layout (location=3) in vec2 texCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 vertexColor;
out vec3 Normal;
out vec3 FragPos;
out vec2 TexCoord;

void main() {
    gl_Position = projection * view * model * vec4(position, 1.0);
    vertexColor = vec4(0.4, 0.4, 0.4, 1.0);
    TexCoord = texCoords;
    Normal = normal;
    FragPos = vec3(model * vec4(position, 1.0));
}
