#version 410

layout (triangles, equal_spacing, ccw) in;

uniform int numTessLevels;
uniform sampler2D vProjectionMaps[4];
uniform sampler2D vNormalMaps[4];
uniform int texRes[4];
uniform int texWidth;
uniform int texHeight;

uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
uniform mat4 projection;      // the projection matrix

uniform float displacementThreshold;

in vec4 VertexColor[];
// received from Tessellation Control Shader - all texture coordinates for the patch vertices
in vec2 TextureCoord[];

in vec3 VertexNormal[];

in ivec2 TessLevel[];

// send to Fragment Shader for coloring
out vec4 vertexColor;
out vec3 normal;
out vec3 FragPos;
out vec2 TexCoord;

vec4 calculateDisplacement(int tessLevel);

void main()
{
    // get patch coordinate
    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;
    float w = gl_TessCoord.z;

    // ----------------------------------------------------------------------
    // retrieve control point texture coordinates
    vec2 t0 = TextureCoord[0];
    vec2 t1 = TextureCoord[1];
    vec2 t2 = TextureCoord[2];

    vec2 texCoord = u * t0 + v * t1 + w * t2;
    TexCoord = texCoord;

    int vertexTessIndex0 = TessLevel[0].x;
    int vertexTessIndex1 = TessLevel[1].x;
    int vertexTessIndex2 = TessLevel[2].x;
    int innerIndex = TessLevel[0].y;

    vec4 displace;
    if (w == 0.0) {
        // On 0-1 edge
        displace = calculateDisplacement(max(vertexTessIndex0, vertexTessIndex1));

        // Reset displacement for corner if its at a lower tessellation level
        // ie its a transition vertex from higher to lower tessellation triangles.
        if (v == 0.0 && vertexTessIndex0 == 0) {
            // v0
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
        else if (u == 0.0 && vertexTessIndex1 == 0) {
            // v1
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
    }
    else if (v == 0.0) {
        // on 2-0 edge
        displace = calculateDisplacement(max(vertexTessIndex0, vertexTessIndex2));
        if (w == 0.0 && vertexTessIndex0 == 0) {
            // v0
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
        else if (u == 0.0 && vertexTessIndex2 == 0) {
            // v2
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
    }
    else if (u == 0.0) {
        // on 1-2 edge
        displace = calculateDisplacement(max(vertexTessIndex1, vertexTessIndex2));
        if (v == 0.0 && vertexTessIndex2 == 0) {
            // v2
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
        else if (w == 0.0 && vertexTessIndex1 == 0) {
            // v1
            displace = vec4(0.0, 0.0, 0.0, 0.0);
        }
    }
    else {
        // Inner vertex
        displace = calculateDisplacement(innerIndex);
    }

    // ----------------------------------------------------------------------
    // retrieve control point position coordinates
    vec4 p0 = gl_in[0].gl_Position;
    vec4 p1 = gl_in[1].gl_Position;
    vec4 p2 = gl_in[2].gl_Position;

    // bilinearly interpolate position coordinate across patch
    vec4 p = p0*u + p1*v + p2*w;
    p += vec4(displace.xyz, 0.0);


    vec4 c0 = VertexColor[0];
    vec4 c1 = VertexColor[1];
    vec4 c2 = VertexColor[2];

//    if (displace.xyz == vec3(0.0, 0.0, 0.0)) {
//        vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
//    }
//    else if (normal == vec3(0.0, 0.0, 0.0)) {
//        vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
//    }
//    else {
    vertexColor = c0*u + c1*v + c2*w;
//    }

//    vertexColor = vec4(normalize(normal), 1.0);

    // ----------------------------------------------------------------------
    // output patch point position in clip space
    gl_Position = projection * view * model * p;
    FragPos = vec3(model * vec4(p.xyz, 1.0));
}

vec4 calculateDisplacement(int tessLevel) {
    int texResolution = texRes[tessLevel];

    int u_int = int(trunc(TexCoord.x * float(texResolution)));
    int v_int = int(trunc(TexCoord.y * float(texResolution)));
    vec4 displace = texelFetch(vProjectionMaps[tessLevel], ivec2(u_int, v_int), 0);
    normal = texture(vNormalMaps[tessLevel], TexCoord).rgb;

    return displace;
}