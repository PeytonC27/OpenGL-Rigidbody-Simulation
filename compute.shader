#version 430 core

// rigidbody data
layout(binding=0, rgba32f) uniform image2D rigidbodyOutTex;  
uniform sampler2D rigidbodyParticles;
uniform sampler2D rigidbodyTex;

uniform float deltaTime;
uniform vec3 gravity;
uniform int particleCount;

float linearDrag = 0.9995;
float angularDrag = 0.99;

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

mat3 quatToMat3(vec4 q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    return mat3(
        1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w,
        2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
        2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y
    );
}

vec4 quaternionMultiply(vec4 q1, vec4 q2) {
    float a1 = q1.x;
    float b1 = q1.y;
    float c1 = q1.z;
    float d1 = q1.w;

    float a2 = q2.x;
    float b2 = q2.y;
    float c2 = q2.z;
    float d2 = q2.w;

    float a3 = a1 * d2 + d1 * a2 + b1 * c2 - c1 * b2;
    float b3 = b1 * d2 + d1 * b2 + c1 * a2 - a1 * c2;
    float c3 = c1 * d2 + d1 * c2 + a1 * b2 - b1 * a2;
    float d3 = d1 * d2 - a1 * a2 - b1 * b2 - c1 * c2;

    return vec4(a3, b3, c3, d3);
}


void main() {
    // rigidbody id
    uint id = gl_GlobalInvocationID.x;  

    // access the row (y value) of the image (which represents a rigidbody)
    ivec2 texCoord = ivec2(0, id); 
    
    // get the rigidbody data from the last tick
    vec3 pos = texelFetch(rigidbodyTex, texCoord + ivec2(0, 0), 0).xyz;
    float mass = 1.0 / texelFetch(rigidbodyTex, texCoord + ivec2(0, 0), 0).w;
    vec3 linearMomentum = texelFetch(rigidbodyTex, texCoord + ivec2(1, 0), 0).xyz;
    float scale = texelFetch(rigidbodyTex, texCoord + ivec2(1, 0), 0).w;
    vec3 angularMomentum = texelFetch(rigidbodyTex, texCoord + ivec2(2, 0), 0).xyz;
    vec4 quaternion = texelFetch(rigidbodyTex, texCoord + ivec2(3, 0), 0);

    vec3 totalForce = vec3(0.0);
    vec3 totalTorque = vec3(0.0);

    // loop through particles belonging to this rigidbody
    for (int i = 0; i < particleCount; i++) {
        ivec2 particleTexCoord = ivec2(i * 4, i);
    
        vec3 relPos = texelFetch(rigidbodyParticles, ivec2(i*4+2, id), 0).xyz;
        vec3 particleForce = texelFetch(rigidbodyParticles, ivec2(i*4+3, id), 0).xyz;

        totalForce += (particleForce * mass) / particleCount;
        totalTorque += (cross(relPos/scale, particleForce) * mass) / particleCount; // torque = r   F
    }

    totalForce += (gravity * mass);
    
    // transformation logic
    linearMomentum = (linearMomentum + totalForce * deltaTime) * linearDrag;
    angularMomentum = (angularMomentum + totalTorque * deltaTime) * angularDrag;

    pos = pos + (linearMomentum / mass) * deltaTime;

    // rotational logic
    mat3 rotationMatrix = quatToMat3(quaternion);

    mat3 inertiaTensor = (1 / 6.0) * mass * 4 * mat3(1.0);
    inertiaTensor = rotationMatrix * inertiaTensor * transpose(rotationMatrix);

    vec3 angularVelocity = inverse(inertiaTensor) * angularMomentum;
    float angularVelocityMagnitude = length(angularVelocity);

    if (angularVelocityMagnitude > 0.000001f) {
        float theta = angularVelocityMagnitude * deltaTime;
        vec3 axis = angularVelocity / angularVelocityMagnitude;
        vec4 quaternionStep = vec4(axis.x * sin(theta / 2.0),
            axis.y * sin(theta / 2.0),
            axis.z * sin(theta / 2.0),
            cos(theta / 2.0));

        quaternion = quaternionMultiply(quaternionStep, quaternion);
    }
    
    // write the newly calculated data
    imageStore(rigidbodyOutTex, texCoord + ivec2(0, 0), vec4(pos, 1.0 / mass)); // pos, inv mass
    imageStore(rigidbodyOutTex, texCoord + ivec2(1, 0), vec4(linearMomentum, scale)); // lin momentum, scale
    imageStore(rigidbodyOutTex, texCoord + ivec2(2, 0), vec4(angularMomentum, 0));
    imageStore(rigidbodyOutTex, texCoord + ivec2(3, 0), quaternion);
}
