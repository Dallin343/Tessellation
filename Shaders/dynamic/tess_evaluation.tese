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
//uniform int numEdgeVertsInBlock;//multires storage
//uniform int numInnerVertsInBlock;//multires storage
uniform int edgeLevelOffsets[3];//multires storage
uniform int innerLevelOffsets[2]; //multires storage
uniform mat4 model;           // the model matrix
uniform mat4 view;            // the view matrix
uniform mat4 projection;      // the projection matrix

uniform float displacementThreshold;

//patch in float gl_TessLevelOuter[4];
//patch in float gl_TessLevelInner[2];

in ivec2 TessLevel[];
in ivec3 EdgeTessIndex[];
patch in int faceID;

// send to Fragment Shader for coloring
out vec4 vertexColor;
out vec3 normal;
out vec3 FragPos;

//vec4 calculateDisplacement(int tessLevel);
bool nearly_equal(float a, float b);
ivec3 calculateVertexIndex(ivec4 tessLevels, vec3 bary);

void main()
{
    // get patch coordinate
    float u = gl_TessCoord.x;
    float v = gl_TessCoord.y;
    float w = gl_TessCoord.z;

    int vertexTessIndex0 = TessLevel[0].x;
    int vertexTessIndex1 = TessLevel[1].x;
    int vertexTessIndex2 = TessLevel[2].x;
    int innerIndex = TessLevel[0].y;

    int edgeTessIndex0 = EdgeTessIndex[0].x;
    int edgeTessIndex1 = EdgeTessIndex[0].y;
    int edgeTessIndex2 = EdgeTessIndex[0].z;

    vec4 tempColor = vec4(0.0, 0.0, 0.0, 1.0);
    FaceAttributes faceAttrs = faceBuffer[gl_PrimitiveID];
    if (gl_PrimitiveID == 147) {
        if (faceAttrs.edgeStart0 == 720 && faceAttrs.edgeStart1 == 722 && faceAttrs.edgeStart2 == -558) {
            tempColor = vec4(1.0, 0.0, 0.0, 1.0);
        }
    }
    ivec4 tessLevel = ivec4(gl_TessLevelOuter[0], gl_TessLevelOuter[1], gl_TessLevelOuter[2], gl_TessLevelInner[0]);
    ivec3 REV = calculateVertexIndex(tessLevel, gl_TessCoord);

    vec4 displace = vec4(0.0, 0.0, 0.0, 0.0);
    vertexColor = vec4(0.0, 0.0, 0.0, 1.0);
    VertexAttributes vertexAttrs;
    if (w == 0.0) {
        // On 0-1 edge
        if (v == 0.0) {
            // v0
            int cornerIndex = faceAttrs.cornerStart0 + vertexTessIndex0;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else if (u == 0.0) {
            // v1
            int cornerIndex = faceAttrs.cornerStart1 + vertexTessIndex1;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else {
            // On the edge
            int start, index;
            if (faceAttrs.edgeStart0 < 0) {
                // Need to go in reverse order
                // Move start to end of this tessellation level
                start = abs(faceAttrs.edgeStart0) + edgeLevelOffsets[edgeTessIndex0 + 1];
                //In this tessellation level, get the vertex at index
                index = start - REV.z;
                vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
            }
            else {
                start = faceAttrs.edgeStart0 + edgeLevelOffsets[edgeTessIndex0];
                //In this tessellation level, get the vertex at index
                index = start + REV.z;
                vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
            }

            vertexAttrs = edgeBuffer[index];

        }
    }
    else if (v == 0.0) {
        // on 2-0 edge
        if (w == 0.0) {
            // v0
            int cornerIndex = faceAttrs.cornerStart0 + vertexTessIndex0;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else if (u == 0.0) {
            // v2
            int cornerIndex = faceAttrs.cornerStart2 + vertexTessIndex2;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else {
            // On the edge
            int start, index;
            if (faceAttrs.edgeStart2 < 0) {
                // Need to go in reverse order
                // Move start to end of this tessellation level
                start = abs(faceAttrs.edgeStart2) + edgeLevelOffsets[edgeTessIndex2 + 1];
                //In this tessellation level, get the vertex at index
                index = start - REV.z;
                vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
            }
            else {
                start = faceAttrs.edgeStart2 + edgeLevelOffsets[edgeTessIndex2];
                //In this tessellation level, get the vertex at index
                index = start + REV.z;
                vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
            }

            vertexAttrs = edgeBuffer[index];
        }
    }
    else if (u == 0.0) {
        // on 1-2 edge
        if (v == 0.0) {
            // v2
            int cornerIndex = faceAttrs.cornerStart2 + vertexTessIndex2;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else if (w == 0.0) {
            // v1
            int cornerIndex = faceAttrs.cornerStart1 + vertexTessIndex1;
            vertexAttrs = cornerBuffer[cornerIndex];
            vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
        }
        else {
            // On the edge
            int start, index;
            if (faceAttrs.edgeStart1 < 0) {
                // Need to go in reverse order
                // Move start to end of this tessellation level
                start = abs(faceAttrs.edgeStart1) + edgeLevelOffsets[edgeTessIndex1 + 1];
//                if (edgeLevelOffsets[edgeTessIndex1 + 1] == 1)
                //In this tessellation level, get the vertex at index
                index = start - REV.z;
                vertexColor = vec4(1.0, 0.0, 0.0, 1.0);
            }
            else {
                start = faceAttrs.edgeStart1 + edgeLevelOffsets[edgeTessIndex1];
                //In this tessellation level, get the vertex at index
                index = start + REV.z;
                vertexColor = vec4(0.0, 0.0, 1.0, 1.0);
            }

            vertexAttrs = edgeBuffer[index];
        }
    }
    else {
        // Inner vertex
        //Move start to beginning of this tessellation level
        int start = faceAttrs.innerStart + innerLevelOffsets[innerIndex];

        //In this tessellation level, get the vertex at index
        int index = start + REV.z;
        vertexAttrs = innerBuffer[index];
        vertexColor = vec4(0.5, 0.5, 0.0, 1.0);
    }


    displace = vertexAttrs.displacement;
    normal = vertexAttrs.normal.xyz;
    if (displace == vec4(0.0, 0.0, 0.0, 0.0)) {
        vertexColor = vec4(0.0, 0.0, 0.0, 1.0);
    }

//    vertexColor = tempColor;

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
//    else {
//        vertexColor = vec4(normalize(normal), 1.0);
//    }
//    else if (normal == vec3(0.0, 0.0, 0.0)) {
//        vertexColor = vec4(0.0, 1.0, 0.0, 1.0);
//    }
//    else {
//    vertexColor = c0*u + c1*v + c2*w;
//    }



    // ----------------------------------------------------------------------
    // output patch point position in clip space
    gl_Position = projection * view * model * p;
    FragPos = vec3(model * vec4(p.xyz, 1.0));
}

//vec4 calculateDisplacement(int tessLevel) {
//    int texResolution = texRes[tessLevel];
//
//    int u_int = int(trunc(TexCoord.x * float(texResolution)));
//    int v_int = int(trunc(TexCoord.y * float(texResolution)));
//    vec4 displace = texelFetch(vProjectionMaps[tessLevel], ivec2(u_int, v_int), 0);
//    normal = texture(vNormalMaps[tessLevel], TexCoord).rgb;
//
//    return displace;
//}

bool nearly_equal(float a, float b) {
    if (a == b) return true;
    return abs(a-b) < 1e-7;
}

ivec3 calculateVertexIndex(ivec4 tessLevels, vec3 bary) {
    int innerLevel = tessLevels.w, outerLevel = tessLevels.x;
    float alpha = bary.x, beta = bary.y, gamma = bary.z;
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