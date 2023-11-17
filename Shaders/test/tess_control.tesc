#version 410

// specify number of control points per patch output
// this value controls the size of the input and output arrays
layout (vertices=3) out;

uniform ivec4 tessellationLevel;

uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
in vec3 vertexNormal[];
out vec3 VertexNormal[];

in vec4 vertexColor[];
out vec4 VertexColor[];
// varying input from vertex shader
in vec2 TexCoord[];
// varying output to evaluation shader
out vec2 TextureCoord[];

void main()
{
    // ----------------------------------------------------------------------
    // pass attributes through
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
    VertexColor[gl_InvocationID] = vertexColor[gl_InvocationID];
    TextureCoord[gl_InvocationID] = TexCoord[gl_InvocationID];
    VertexNormal[gl_InvocationID] = vertexNormal[gl_InvocationID];

    // ----------------------------------------------------------------------
    // invocation zero controls tessellation levels for the entire patch
    if (gl_InvocationID == 0)
    {
        gl_TessLevelOuter[0] = tessellationLevel.x;
        gl_TessLevelOuter[1] = tessellationLevel.y;
        gl_TessLevelOuter[2] = tessellationLevel.z;

        gl_TessLevelInner[0] = tessellationLevel.w;
    }
}
