#version 410

layout (triangles, equal_spacing, ccw) in;

uniform sampler2D vProjectionMap;  // the texture corresponding to our projection map
uniform int texWidth;
uniform int texHeight;
uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
uniform mat4 projection;      // the projection matrix

in vec4 VertexColor[];
// received from Tessellation Control Shader - all texture coordinates for the patch vertices
in vec2 TextureCoord[];

in vec3 VertexNormal[];

// send to Fragment Shader for coloring
//out float Height;
out vec4 vertexColor;
out vec3 normal;
out vec3 FragPos;

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
    vec4 vProject = texture(vProjectionMap, texCoord);
    int u_int = int(trunc(texCoord.x * float(texWidth)));
    int v_int = int(trunc(texCoord.y * float(texHeight)));
    vec4 test = texelFetch(vProjectionMap, ivec2(u_int, v_int), 0);

    // ----------------------------------------------------------------------
    // retrieve control point position coordinates
    vec4 p0 = gl_in[0].gl_Position;
    vec4 p1 = gl_in[1].gl_Position;
    vec4 p2 = gl_in[2].gl_Position;

    // compute patch surface normal
//    vec4 uVec = p1 - p0;
//    vec4 vVec = p2 - p0;
//    vec4 normal = normalize( vec4(cross(vVec.xyz, uVec.xyz), 0) );

    // bilinearly interpolate position coordinate across patch
    vec4 p = p0*u + p1*v + p2*w;
//    p += vec4(vProject.xyz, 0.0);
    float testLen = length(test.xyz);
    float thresh = 60.0;
//    if (testLen < thresh) {
        p += vec4(test.xyz, 0.0);
//    }


    vec4 c0 = VertexColor[0];
    vec4 c1 = VertexColor[1];
    vec4 c2 = VertexColor[2];
//    vertexColor = normalize(vec4(u, v, w, 1.0));
/*    if (test.xyz == vProject.xyz) {
        vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
    } else */if (test.xyz == vec3(0.0, 0.0, 0.0)) {
        vertexColor = vec4(0.0, 0.0, 0.0, 1.0);
    } else {
//        vertexColor = vec4(0.0, 0.0, 0.3, 1.0);
        vertexColor = c0*u + c1*v + c2*w;
//        vertexColor = normalize(vec4(test.xyz, 1.0));
    }

    vec3 n0 = VertexNormal[0];
    vec3 n1 = VertexNormal[1];
    vec3 n2 = VertexNormal[2];

    vec4 uVec = p1 - p0;
    vec4 vVec = p2 - p0;
    normal = normalize(n0*u + n1*v + n2*w);

    // ----------------------------------------------------------------------
    // output patch point position in clip space
    gl_Position = projection * view * model * p;
    FragPos = vec3(model * vec4(p.xyz, 1.0));
}