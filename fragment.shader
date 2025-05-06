#version 430 core

in vec3 fragNorm;
in vec3 fragPos;
in vec3 fragColor;

uniform vec3 lightPos;
uniform vec3 cameraPos;

uniform vec3 diffColor = vec3(0.5, 0.5, 0.5);
uniform vec3 ambientColor = vec3(0.1, 0.1, 0.1);

out vec4 color;

void main() {
    vec3 lightDir = normalize(lightPos - fragPos);

    // diffusion
    vec3 diffusion = max(dot(fragNorm, lightDir), 0.0) * fragColor;

    vec3 resultingColor = diffusion + (fragColor / 10.0);

    color = vec4(resultingColor, 1);
}