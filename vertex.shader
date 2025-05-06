#version 430 core

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 norm;

uniform mat4 mvp;
uniform mat4 normMat;
uniform mat4 projectionMat;

uniform sampler2D rigidbodyTex; // bound as texture2D

out vec3 fragPos;
out vec3 fragNorm;
out vec3 fragColor;

//Optional: quaternion to rotation matrix helper
mat3 quatToMat3(vec4 q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    return mat3(
        1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w,
        2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
        2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y
    );
}

void main() {
    // get rigidbody info
    int cubeID = gl_InstanceID;

    vec3 position = texelFetch(rigidbodyTex, ivec2(0, cubeID), 0).xyz;
    float scale = texelFetch(rigidbodyTex, ivec2(1, cubeID), 0).w;
    vec4 orientation = texelFetch(rigidbodyTex, ivec2(3, cubeID), 0);
    vec4 col = texelFetch(rigidbodyTex, ivec2(4, cubeID), 0);

    // calculate fragment shader info
    fragNorm = normalize(quatToMat3(orientation) * norm);
    fragPos = vec3(projectionMat * vec4(pos * scale, 1.0));
    fragColor = vec3(col);

    gl_Position = mvp * vec4(quatToMat3(orientation) * pos * scale - position, 1.0);
}