# OpenGL Rigidbody Simulation
A simple attempt at a rigidbody simulation using OpenGL, GLSL, and C++ for my interactive graphics class. [This article](https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-29-real-time-rigid-body-simulation-gpus) was referenced in this implementation.

### Representing Rigidbodies
Each rigidbody is represented by 7 variables: position, mass, linear momentum, scale, angular momentum, orientation, and color. To work on the GPU, we store these values in a 2D texture as color values in a 32-bit floating point. I chose to store the values like so:
```
texel0 - position (vec3) and inverse mass (float)
texel1 - linear momentum (vec3) and the object's scale (float)
texel2 - angular momentum (vec3) and 0
texel3 - orientation (quaternion, or vec4)
texel4 - color (vec4)
```

### Particles
Each rigidbody we will represent as a bunch of particles (like in the real world, but far less realistic). My implementation represents objects as cubes (think of it as having a box collider in Unity), which is crude, unrealistic, but very easy to implement. A compute shader will take each rigidbody, generate a specific amount of particles for each of them, and store them in a 2D texture. Each particle is represented in the texture like so:

```
texel0 - position (vec3) and ID (int)
texel1 - velocity (vec3) and 1
texel2 - relative position (vec3) and 0
texel3 - applied forces (vec3) and 0
```

### Physics Tick
In order to move the objects, we need to tick time forwards on the GPU, calculating new linear/angular velocities, transformations, and changes in orientation. To do so, we need some derivatives with respect to time, and we can achieve that with two time slices (a before and current snapshot). So we need to create two time snippets to send to the GPU, one that represents the "before" physics and one the represents the "current" physics.

We need two 2D textures, each storing all the rigidbody information. A technique known as "flip-flopping" is performed to simulate a physics tick:
```
read data from A
move physics forwards, store it in B
swap data in A and B
repeat
```

This is done on the CPU, where we swap a pointer to A or B respectively each tick.

### Collision

To tackle collision, the article talks of a method were we generate a grid, assign each particle to a grid coordinate, then perform checks around said grid coordinate to see if any particles are close enough to trigger a collision.

We take a particle and its index, calculate its position, assign it a grid index, then save it in a 2D texture that will act as a hash table. Because our 2D textures store 32-bit floats, we can only store 4 particles in each index, which should be fine according to the article above. After running each particle through this, for each particle, we check surrounding grid indices and see if any particles are close enough to trigger a collision (given the other particle is apart of another rigidbody), then update the "force" component of each particle; which will later be used for physics tick calculation.

### Results
The results are not fully realistic. There are issues with rigidbodies sticking together instead of properly colliding, and sometime particles will be sent away from each other at light speed. I assume these have to do with my relative position calculations and that physics calculations rely on delta time, not a fixed delta time, but the results were still enough to display a working demo.
