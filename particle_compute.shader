#version 430 core

// rigidbody data
layout(binding=0, rgba32f) uniform image2D particleOutputTex; 
uniform sampler2D rigidbody;
uniform int density;


layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

mat3 quatToMat3(vec4 q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    return mat3(
        1 - 2*y*y - 2*z*z,   2*x*y - 2*z*w,     2*x*z + 2*y*w,
        2*x*y + 2*z*w,       1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
        2*x*z - 2*y*w,       2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y
    );
}

void main() {
    // rigidbody id
    uint id = gl_GlobalInvocationID.x;  

    // access the row (y value) of the image (which represents a rigidbody)
    ivec2 texCoord = ivec2(0, id); 
    
    // get the data from the center of mass
    vec3 pos = texelFetch(rigidbody, texCoord + ivec2(0, 0), 0).xyz;
    float mass = 1.0 / texelFetch(rigidbody, texCoord + ivec2(0, 0), 0).w;

    mat3 inertiaTensor = (1 / 6.0) * mass * 4 * mat3(1.0);

    vec3 velocity = texelFetch(rigidbody, texCoord + ivec2(1, 0), 0).xyz / mass;
    vec3 angularVelocity = inverse(inertiaTensor) * texelFetch(rigidbody, texCoord + ivec2(2, 0), 0).xyz;
    float scale = texelFetch(rigidbody, texCoord + ivec2(1, 0), 0).w;
    vec4 quaternion = texelFetch(rigidbody, texCoord + ivec2(3, 0), 0);
    

    // assume object is cube and generate particles
    float xyzOffset = (1.0 * scale / density);
    
    float bound = scale * 0.5f;
    float start = -bound + xyzOffset/2;
    float end = bound - xyzOffset/2 + 0.001;
    
    int particleIndex = 0;
    for (float y = start; y < end; y += xyzOffset) {
        for (float x = start; x < end; x += xyzOffset) {
            for (float z = start; z < end; z += xyzOffset) {
                vec3 relativePos = quatToMat3(quaternion) * vec3(x,y,z);
                vec3 particlePos = pos + relativePos;


                imageStore(particleOutputTex, ivec2(particleIndex+0, id), vec4(particlePos, particleIndex/4 + density * density * density * id + 1));
                imageStore(particleOutputTex, ivec2(particleIndex+1, id), vec4(velocity + angularVelocity * relativePos, 1));
                imageStore(particleOutputTex, ivec2(particleIndex+2, id), vec4(relativePos, 0));
                imageStore(particleOutputTex, ivec2(particleIndex+3, id), vec4(0,0,0,0)); // for collisions
                particleIndex += 4;
            }
        }
    }
 
}
