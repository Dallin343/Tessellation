#version 410 core

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform bool inWireframe;

out vec4 FragColor;
in vec4 vertexColor; // the input variable from the vertex shader (same name and same type)
in vec2 TexCoord;

in vec3 Normal;
in vec3 FragPos;

void main()
{
    if (inWireframe) {
        FragColor = vertexColor;
        return;
    }
    //Ambient
    float ambientStrength = 0.1;
    vec3 lightColor = vec3(1.0, 1.0, 1.0);
    vec3 ambient = ambientStrength * lightColor;

    //Diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    //Specular
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);

    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
    vec3 specular = specularStrength * spec * lightColor;


    vec3 result = (ambient + diffuse + specular) * vertexColor.xyz;
    FragColor = vec4(result, 1.0);
}
