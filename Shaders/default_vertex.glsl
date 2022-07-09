#version 410

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec4 vertexColor;

void main() {
    gl_Position = projection * view * model * vec4(position, 1.0);
    vertexColor = vec4(normal, 1.0);
    //vertexColor = vec4(0.0,0.0,0.0, 1.0);
}
