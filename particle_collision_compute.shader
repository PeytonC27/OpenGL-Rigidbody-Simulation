#version 430 core

layout(binding = 0, rgba32f) uniform image2D particles; 
layout(binding = 1, rgba32f) uniform image2D gridStructure;
layout(binding = 2, rgba32f) uniform image2D gridCounter;

uniform int totalParticles;
uniform float particleDiameter;
uniform int particlesPerCube;
uniform int particleDataSize;
uniform int gridSize;

uniform vec3 smallestGridCoordinates;

uniform float springCoeff;
uniform float dampingCoeff;

layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;

/// calculate the collision force between two particles
vec4 calculateCollisionForce(vec3 ap, vec3 av, vec3 bp, vec3 bv) {
    vec3 relativeVelocity = bv - av;
    vec3 relativePosition = bp - ap;

    float dist = length(relativePosition);
    vec3 normal = normalize(relativePosition);

    vec3 fis = -springCoeff * (particleDiameter - dist) * normal;
    vec3 fid = dampingCoeff * normalize(relativeVelocity);
    vec3 fit = springCoeff * (normalize(relativeVelocity)- dot(normalize(relativeVelocity), normal) * normal);

    vec3 sum = fis + fid + fit;
    return vec4(sum, 0);
}

// get the position of a particle given its index
vec4 getPosFromParticleIndex(int particleIndex) {
    return imageLoad(
        particles,
        ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)),
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)))
    );
}

// get the velocity of a particle given its index
vec4 getVelFromParticleIndex(int particleIndex) {
    return imageLoad(
        particles,
        ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)) + 1,
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)))
    );
}

// get the grid index of a particle given its index
ivec2 getParticleCoord(int particleIndex) {
    return ivec2(int((particleIndex * particleDataSize) % (particlesPerCube * particleDataSize)) + 1,
        int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize)));
}

// get the rigidbody id of a particle given its index
int getRigidbody(int particleIndex) {
    return int((particleIndex * particleDataSize) / (particlesPerCube * particleDataSize));
}

// check if the distance between two particles is valid
bool checkDistance(vec3 posA, vec3 posB) {
    return abs(length(posA-posB)) < particleDiameter/3;
}

void main() {
    // get initial data
    int px = int((gl_GlobalInvocationID.x * particleDataSize) % (particlesPerCube * particleDataSize));
    int py = int((gl_GlobalInvocationID.x * particleDataSize) / (particlesPerCube * particleDataSize));

    ivec2 particleCoord = ivec2(px, py);
    ivec2 particleForceCoord = ivec2(px+3, py);

    vec4 positionData = getPosFromParticleIndex(int(gl_GlobalInvocationID.x));
    vec4 velocityData = getVelFromParticleIndex(int(gl_GlobalInvocationID.x));

    vec3 pos = positionData.xyz;
    vec3 vel = velocityData.xyz;

    vec4 force = vec4(0);

    vec3 gridIndex = floor((pos - smallestGridCoordinates) / particleDiameter);
    gridIndex = clamp(gridIndex, vec3(0), vec3(gridSize - 1));
    ivec2 coord = ivec2(int(gridIndex.x) + int(gridIndex.y) * gridSize, int(gridIndex.z));

    // check all 27 grid locations around this particle
    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            for (int z = -1; z <= 1; z++) {
                vec3 newIndex = vec3(gridIndex.x+x, gridIndex.y+y, gridIndex.z+z);
                ivec2 structureIndex = ivec2(int(newIndex.x) + int(newIndex.y) * gridSize, int(newIndex.z));

                if (newIndex.x < 0 || newIndex.x >= gridSize || 
                    newIndex.y < 0 || newIndex.y >= gridSize || 
                    newIndex.z < 0 || newIndex.z >= gridSize)
                    continue;

                vec4 entry = imageLoad(gridStructure, structureIndex);
                int count = int(imageLoad(gridCounter, structureIndex).r);

                // we have a collision
                if (count != 0) {
                    if (count >= 1 && int(entry.x) != gl_GlobalInvocationID.x && getRigidbody(int(entry.x)) != getRigidbody(int(gl_GlobalInvocationID.x))) {
                        vec4 newDataPos = getPosFromParticleIndex(int(entry.x));
                        vec4 newDataVel = getVelFromParticleIndex(int(entry.x));

                        if (checkDistance(pos, newDataPos.xyz))
                            force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
                    }
                    if (count >= 2 && int(entry.y) != gl_GlobalInvocationID.x && getRigidbody(int(entry.y)) != getRigidbody(int(gl_GlobalInvocationID.x))) {
                        vec4 newDataPos = getPosFromParticleIndex(int(entry.y));
                        vec4 newDataVel = getVelFromParticleIndex(int(entry.y));

                        if (checkDistance(pos, newDataPos.xyz))
                            force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
                    }
                    if (count >= 3 && int(entry.z) != gl_GlobalInvocationID.x && getRigidbody(int(entry.z)) != getRigidbody(int(gl_GlobalInvocationID.x))) {
                        vec4 newDataPos = getPosFromParticleIndex(int(entry.z));
                        vec4 newDataVel = getVelFromParticleIndex(int(entry.z));

                        if (checkDistance(pos, newDataPos.xyz))
                            force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
                    }
                    if (count == 4 && int(entry.w) != gl_GlobalInvocationID.x && getRigidbody(int(entry.w)) != getRigidbody(int(gl_GlobalInvocationID.x))) {
                        vec4 newDataPos = getPosFromParticleIndex(int(entry.w));
                        vec4 newDataVel = getVelFromParticleIndex(int(entry.w));

                        if (checkDistance(pos, newDataPos.xyz))
                            force += calculateCollisionForce(pos, vel, newDataPos.xyz, newDataVel.xyz);
                    }
                }
            }
        }
    }

    imageStore(particles, particleForceCoord, force);
}