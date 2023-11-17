#version 410

layout (triangles, equal_spacing, ccw) in;

uniform sampler2D vProjectionMap;  // the texture corresponding to our projection map
uniform sampler2D vNormalMap; // Normal map for tessellated vertices;
uniform int texRes;

uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
uniform mat4 projection;      // the projection matrix


in vec4 VertexColor[];
// received from Tessellation Control Shader - all texture coordinates for the patch vertices
in vec2 TextureCoord[];

in vec3 VertexNormal[];

// send to Fragment Shader for coloring
out vec4 vertexColor;
out vec3 normal;
out vec3 FragPos;
out vec2 TexCoord;


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

    // ----------------------------------------------------------------------
    // retrieve control point position coordinates
    vec4 p0 = gl_in[0].gl_Position;
    vec4 p1 = gl_in[1].gl_Position;
    vec4 p2 = gl_in[2].gl_Position;

    vec4 uVec = p1 - p0;
    vec4 vVec = p2 - p0;
    normal = normalize(cross(vVec.xyz, uVec.xyz));

    // bilinearly interpolate position coordinate across patch
    vec4 p = p0*u + p1*v + p2*w;


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

    vertexColor = vec4(normalize(normal), 1.0);

    // ----------------------------------------------------------------------
    // output patch point position in clip space
    gl_Position = projection * view * model * p;
    FragPos = vec3(model * vec4(p.xyz, 1.0));
}