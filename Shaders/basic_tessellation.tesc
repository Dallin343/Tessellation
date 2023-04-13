#version 410

// specify number of control points per patch output
// this value controls the size of the input and output arrays
layout (vertices=3) out;

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

    // ----------------------------------------------------------------------
    // invocation zero controls tessellation levels for the entire patch
    if (gl_InvocationID == 0)
    {
        gl_TessLevelOuter[0] = 5;
        gl_TessLevelOuter[1] = 5;
        gl_TessLevelOuter[2] = 5;

        gl_TessLevelInner[0] = 5;
    }
}
