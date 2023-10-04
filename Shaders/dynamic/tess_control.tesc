#version 410

// specify number of control points per patch output
// this value controls the size of the input and output arrays
layout (vertices=3) out;

uniform ivec4 tessellationLevel;
uniform ivec4 tessellationLevels[4];
uniform float tessellationLevelDistances[4];

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

out ivec2 TessLevel[];

int getTessellationLevelIndex(float distance) {
    if (distance >= tessellationLevelDistances[0]) {
        return 0;
    }
    else if (distance >= tessellationLevelDistances[1]) {
        return 1;
    }
    else if (distance >= tessellationLevelDistances[2]) {
        return 2;
    }
    else if (distance >= tessellationLevelDistances[3]) {
        return 3;
    }
    return 3;
}

void main()
{
    // ----------------------------------------------------------------------
    // pass attributes through
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;
    VertexColor[gl_InvocationID] = vertexColor[gl_InvocationID];
    TextureCoord[gl_InvocationID] = TexCoord[gl_InvocationID];
    VertexNormal[gl_InvocationID] = vertexNormal[gl_InvocationID];

    vec4 eyeSpacePos = view * model * gl_in[gl_InvocationID].gl_Position;

    float distance = -eyeSpacePos.z;

    int tessLevelIndex = getTessellationLevelIndex(distance);
    TessLevel[gl_InvocationID] = ivec2(tessLevelIndex, 0);

    // ----------------------------------------------------------------------
    // invocation zero controls tessellation levels for the entire patch
    if (gl_InvocationID == 0)
    {

        // ----------------------------------------------------------------------
        // Step 2: transform each vertex into eye space
        vec4 eyeSpacePos0 = view * model * gl_in[0].gl_Position;
        vec4 eyeSpacePos1 = view * model * gl_in[1].gl_Position;
        vec4 eyeSpacePos2 = view * model * gl_in[2].gl_Position;

        // ----------------------------------------------------------------------
        // Step 3: "distance" from camera
        float distance0 = abs(eyeSpacePos0.z);
        float distance1 = abs(eyeSpacePos1.z);
        float distance2 = abs(eyeSpacePos2.z);

        int vertex0TessLevelIndex = getTessellationLevelIndex(distance0);
        int vertex1TessLevelIndex = getTessellationLevelIndex(distance1);
        int vertex2TessLevelIndex = getTessellationLevelIndex(distance2);


        // ----------------------------------------------------------------------
        // Step 4: interpolate edge tessellation level based on closer vertex
        int tessLevelIndex0 = max(vertex0TessLevelIndex, vertex2TessLevelIndex);
        int tessLevelIndex1 = max(vertex0TessLevelIndex, vertex1TessLevelIndex);
        int tessLevelIndex2 = max(vertex1TessLevelIndex, vertex2TessLevelIndex);


        gl_TessLevelOuter[0] = tessellationLevels[tessLevelIndex2].x;
        gl_TessLevelOuter[1] = tessellationLevels[tessLevelIndex0].y;
        gl_TessLevelOuter[2] = tessellationLevels[tessLevelIndex1].z;

        int innerIndex = max(tessLevelIndex0, max(tessLevelIndex1, tessLevelIndex2));
        gl_TessLevelInner[0] = tessellationLevels[innerIndex].w;

        TessLevel[gl_InvocationID] = ivec2(tessLevelIndex, innerIndex);
    }
}
