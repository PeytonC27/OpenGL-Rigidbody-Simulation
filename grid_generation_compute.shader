#version 430 core

layout(binding = 0, rgba32f) uniform image2D particles; 
layout(binding = 1, rgba32f) uniform image2D gridStructure;
layout(binding = 2, rgba32f) uniform image2D gridCounter;

uniform sampler2D rigidbody;

uniform int totalParticles;
uniform float particleDiameter;
uniform int particlesPerCube;
uniform int particleDataSize;
uniform int gridSize;

uniform vec3 smallestGridCoordinates;

uniform float springCoeff;
uniform float dampingCoeff;

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

vec4 calculateCollisionForce(vec3 ap, vec3 av, vec3 bp, vec3 bv) {
    vec3 relativeVelocity = bv - av;
    vec3 relativePosition = bp - ap;

    float dist = length(relativePosition);
    vec3 normal = normalize(relativePosition);

    vec3 fis = -springCoeff * (particleDiameter - dist) * normal;
    vec3 fid = dampingCoeff * relativeVelocity;
    vec3 fit = springCoeff * (relativeVelocity - dot(relativeVelocity, normal) * normal);

    //return fis + fit + fid;
    vec3 sum = fis + fid + fit;
    return vec4(sum, 0);
}

vec4 getPosFromParticleIndex(int particleIndex) {
    return imageLoad(
        particles,
        ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)),
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)))
    );
}

vec4 getVelFromParticleIndex(int particleIndex) {
    return imageLoad(
        particles,
        ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)) + 1,
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)))
    );
}

ivec2 getParticleCoord(int particleIndex) {
    return ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)) + 1,
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)));
}

int getRigidbody(int particleIndex) {
    return int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize));
}

void main() {
    for (int i = 0; i < totalParticles; i++) {
        int px = int((i * particleDataSize) % (particlesPerCube * particleDataSize));
        int py = int((i * particleDataSize) / (particlesPerCube * particleDataSize));

        vec4 positionData = getPosFromParticleIndex(i);
        vec4 velocityData = getVelFromParticleIndex(i);

        vec3 pos = positionData.xyz;
        vec3 vel = velocityData.xyz;

        vec3 gridIndex = floor((pos - smallestGridCoordinates) / particleDiameter);
        gridIndex = clamp(gridIndex, vec3(0), vec3(gridSize - 1));
        ivec2 coord = ivec2(int(gridIndex.x) + int(gridIndex.y) * gridSize, int(gridIndex.z));


        int count = int(imageLoad(gridCounter, coord).r);

        if (count < 4) {
            vec4 entry = imageLoad(gridStructure, coord);
            if (count == 0) entry.x = i;
            else if (count == 1) entry.y = i;
            else if (count == 2) entry.z = i;
            else if (count == 3) entry.w = i;

            imageStore(gridStructure, coord, entry);
            imageStore(gridCounter, coord, vec4(count+1,0,0,0));
        }
        
    }

    // for (int i = 0; i < totalParticles; i++) {
    //     int px = int((i * particleDataSize) % (particlesPerCube * particleDataSize));
    //     int py = int((i * particleDataSize) / (particlesPerCube * particleDataSize));

    //     ivec2 particleCoord = ivec2(px, py);
    //     ivec2 particleForceCoord = ivec2(px+3, py);

    //     vec4 positionData = getPosFromParticleIndex(i);
    //     vec4 velocityData = getVelFromParticleIndex(i);

    //     vec3 pos = positionData.xyz;
    //     vec3 vel = velocityData.xyz;

    //     vec4 force = imageLoad(particles, particleForceCoord);

    //     vec3 gridIndex = floor((pos - smallestGridCoordinates) / particleDiameter);
    //     int gridStructureIndex = int(gridIndex.x) + int(gridIndex.y) * gridSize + int(gridIndex.z) * gridSize * gridSize;

    //     for (int x = -1; x <= 1; x++) {
    //         for (int y = -1; y <= 1; y++) {
    //             for (int z = -1; z <= 1; z++) {
    //                 vec3 newIndex = vec3(gridIndex.x+x, gridIndex.y+y, gridIndex.z+z);
    //                 int structureIndex = int(newIndex.x) + int(newIndex.y) * gridSize + int(newIndex.z) * gridSize * gridSize;

    //                 vec4 entry = imageLoad(gridStructure, ivec2(structureIndex, 0));
    //                 int count = int(imageLoad(gridCounter, ivec2(structureIndex, 0)).r);

    //                 // we have a collision
    //                 if (count != 0) {
    //                     if (count >= 1 && int(entry.x) != i && getRigidbody(int(entry.x)) != py) {
    //                         vec4 newDataPos = getPosFromParticleIndex(int(entry.x));
    //                         vec4 newDataVel = getVelFromParticleIndex(int(entry.x));

    //                         force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
    //                     }
    //                     if (count >= 2 && int(entry.y) != i && getRigidbody(int(entry.y)) != py) {
    //                         vec4 newDataPos = getPosFromParticleIndex(int(entry.y));
    //                         vec4 newDataVel = getVelFromParticleIndex(int(entry.y));

    //                         force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
    //                     }
    //                     if (count >= 3 && int(entry.z) != i && getRigidbody(int(entry.z)) != py) {
    //                         vec4 newDataPos = getPosFromParticleIndex(int(entry.z));
    //                         vec4 newDataVel = getVelFromParticleIndex(int(entry.z));

    //                         force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
    //                     }
    //                     if (count == 4 && int(entry.w) != i && getRigidbody(int(entry.w)) != py) {
    //                         vec4 newDataPos = getPosFromParticleIndex(int(entry.w));
    //                         vec4 newDataVel = getVelFromParticleIndex(int(entry.w));

    //                         force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
    //                     }
    //                 }

    //                 /// no collision, ignore
    //             }
    //         }
    //     }

    //     imageStore(particles, particleForceCoord, force);
    // }
}