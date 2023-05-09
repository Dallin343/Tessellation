#version 410 core

//uniform sampler2D vProjectionMap;

out vec4 FragColor;
in vec4 vertexColor; // the input variable from the vertex shader (same name and same type)
in vec2 TexCoord;

void main()
{
//    FragColor = vec4(texture(vProjectionMap, TexCoord).rgb, 1.0);
    FragColor = vertexColor;
}
