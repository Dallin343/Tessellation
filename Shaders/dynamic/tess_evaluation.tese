#version 430

layout (triangles, equal_spacing, ccw) in;

struct VertexAttributes {
    vec4 displacement;
    vec4 normal;
};

struct FaceAttributes {
    int cornerStart0;
    int cornerStart1;
    int cornerStart2;
    int innerStart;
    int edgeStart0;
    int edgeStart1;
    int edgeStart2;
};

layout(std430, binding = 1) buffer CornerBuffer {
    VertexAttributes cornerBuffer[];
};

layout(std430, binding = 2) buffer EdgeBuffer {
    VertexAttributes edgeBuffer[];
};

layout(std430, binding = 3) buffer InnerBuffer {
    VertexAttributes innerBuffer[];
};

layout(std430, binding = 4) buffer FaceBuffer {
    FaceAttributes faceBuffer[];
};

uniform int numTessLevels;
uniform int edgeLevelOffsets[2];
uniform int innerLevelOffsets[2];
uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
uniform mat4 projection;      // the projection matrix

uniform float displacementThreshold;

patch in float gl_TessLevelOuter[4];
patch in float gl_TessLevelInner[2];

in ivec2 TessLevel[];
patch in int faceID;

// send to Fragment Shader for coloring
out vec3 normal;
out vec3 FragPos;

vec4 calculateDisplacement(int tessLevel);
ivec3 calculateVertexIndex(ivec4 tessLevel, vec3 bary);

void main()
{
    // get patch coordinate
    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;
    float w = gl_TessCoord.z;

    FaceAttributes faceAttrs = faceBuffer[faceID];
    ivec4 tessLevel = ivec4(gl_TessLevelOuter[0], gl_TessLevelOuter[1], gl_TessLevelOuter[2], gl_TessLevelInner[0]);
    ivec3 REV = calculateVertexIndex(tessLevel, gl_TessCoord);

    

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


//    vec4 c0 = VertexColor[0];
//    vec4 c1 = VertexColor[1];
//    vec4 c2 = VertexColor[2];

//    if (displace.xyz == vec3(0.0, 0.0, 0.0)) {
//        vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
//    }
//    else if (normal == vec3(0.0, 0.0, 0.0)) {
//        vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
//    }
//    else {
//    vertexColor = c0*u + c1*v + c2*w;
//    }

    vertexColor = vec4(normalize(normal), 1.0);

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

bool nearly_equal(float a, float b) {
    if (a == b) return true;
    return abs(a-b) < 1e-7;
}

ivec3 calculateVertexIndex(ivec4 tessLevel, ivec3 bary) {
    int innerLevel = tessLevels.w, outerLevel = tessLevels.x;
    int alpha = bary.x, beta = bary.y, gamma = bary.z;
    float b_min = min(alpha, min(beta, gamma));
    int ring = int(round(1.5 * innerLevel * b_min));

    int edge;
    float b_prime;
    if (nearly_equal(b_min, alpha)) {
        // On edge 1 side of triangle between v1, v2
        edge = 1;
        b_prime = gamma;
        if (nearly_equal(b_min, beta)) {
            //Gamma corner, we need to change edge to reflect that
            edge = 2;
            b_prime = alpha;
        }
    }
    else if (nearly_equal(b_min, beta)) {
        // On edge 2 side of triangle between v2, v0
        edge = 2;
        b_prime = alpha;
        if (nearly_equal(b_min, gamma)) {
            //alpha corner
            edge = 0;
            b_prime = beta;
        }
    }
    else if (nearly_equal(b_min, gamma)) {
        // On edge 0 side of triangle between v0, v1
        edge = 0;
        b_prime = beta;
        if (nearly_equal(b_min, beta)) {
            //beta corner
            edge = 1;
            b_prime = gamma;
        }
    }

    int T = ring == 0 ? outerLevel : innerLevel;
    int vertsPerEdge = T - 2*ring;

    //If vertsPerEdge is 0, then this is the centermost/last inner vertex
    float vertex = vertsPerEdge == 0 ? 0.0 : (b_prime / (1.0 - 3.0*b_min)) * float(vertsPerEdge);
    int vertexIndexOnEdge = int(vertex);

    int relativeVertexIndex;
    if (ring != 0) {
        // This is an inner face vertex
        // Add any outer ring indices to start at the next sequential index
        int ringRelativeStartIndex = 0;
        for (int i = 1; i < ring; i++) {
            ringRelativeStartIndex += 3 * (T - 2 * i); // Add number of vertices in previous ring
        }

        // If all barycentric components are equal, this is the center/last inner vertex
        if (nearly_equal(alpha, beta) && nearly_equal(beta, gamma)) {
            relativeVertexIndex = ringRelativeStartIndex;
        }
        else {
            // Get the index for the start of this edge
            ringRelativeStartIndex += edge * vertsPerEdge;
            //Add vertex index on this edge
            relativeVertexIndex = ringRelativeStartIndex + vertexIndexOnEdge;
        }
    }
    else {
        // This vertex is on an edge, so it is just the index on that edge.
        relativeVertexIndex = vertexIndexOnEdge - 1;
    }

    return ivec3(ring, edge, relativeVertexIndex);
}