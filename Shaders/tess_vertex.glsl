#version 410

layout (location=0) in vec3 position;
layout (location=1) in vec3 normal;
layout (location=2) in vec2 texCoords;

//uniform sampler2D vProjectionMap;
//uniform mat4 model;
//uniform mat4 view;
//uniform mat4 projection;

out vec4 vertexColor;
out vec2 TexCoord;

void main() {
//    gl_Position = projection * view * model * vec4(position, 1.0);
    gl_Position = vec4(position, 1.0);
//    vertexColor = texture(vProjectionMap, texCoords);
    vertexColor = vec4(normal, 1.0);
    TexCoord = texCoords;
//    vertexColor = vec4(0.0,0.0,0.0, 1.0);
}
