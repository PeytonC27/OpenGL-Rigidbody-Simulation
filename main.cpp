#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/gl.h>
#include "cyTriMesh.h"
#include "cyMatrix.h"
#include "cyGL.h"

#include <iostream>
#include <algorithm>

#define PI 3.1415926f

// ==================== IMPORTANT CONSTANTS FOR SETUP ==================== //
const int SCREEN_WIDTH = 1000, SCREEN_HEIGHT = 1000; // screen dimensions
const int CUBE_PARTICLE_DENSITY = 3; // how densely we want to split the object (1 = 1x1x1, 3 = 3x3x3, even numbers do not work)
const int CUBE_PARTICLE_COUNT = CUBE_PARTICLE_DENSITY * CUBE_PARTICLE_DENSITY * CUBE_PARTICLE_DENSITY;

const int RIGIDBODY_DATA_SIZE = 5; // storing position/mass, linear momentum, angular momentum, and orientation (quaternion)
const int PARTICLE_DATA_SIZE = 4; // storing position, velocity, relative position, and grid index

const int WORLD_GRID_SIZE = 50; // this x this x this is the dimension of the world

const float SPRING_COEFF = 5000;
const float DAMPING_COEFF = 0.5;

const float CUBE_SCALE = 3;

// ==================== CONSTANTS TO TINKER WITH ==================== //
const float ZFAR = 500;
const float GRAVITY = 0;

const int CUBES_TO_SPAWN = 200; // number of objects to spawn in the scene

const float INITIAL_MOMENTUM_X_RANGE = 20;
const float INITIAL_MOMENTUM_Z_RANGE = 20;

const float MIN_Y_INITIAL_MOMENTUM = 300;
const float MAX_Y_INITIAL_MOMENTUM = 500;

const float MIN_OBJECT_SCALE = 1.0f;
const float MAX_OBJECT_SCALE = 2.0f;

const float Y_SPAWN_MIN = -20;
const float Y_SPAWN_MAX = 20;

const float MAX_INITIAL_ANGULAR_MOMENTUM = 10;

const bool EXPLODE = true;


cy::GLSLProgram prog;
GLuint computeShaderProgram, particleGenProgram, gridGenProgram, particleCollisionProgram;
cy::TriMesh cubeMesh;
GLuint cubeVAO, cubePositions, cubeNormals, cubeTXCs, cubeEBO;

GLuint rigidbodyTexA, rigidbodyTexB, particles, gridDataStructure, gridCountingStructure;
bool flipFlop = false;

float camXAngle = PI, camYAngle = PI, camDist = 100;
bool LMB = false, RMB = false;
int lastMousePosX, lastMousePosY;

cy::Matrix4f mvpMatrix, viewMatrix;

float oldT;

float ranrange(float min, float max);
cy::Vec4f randomQuaternion();

bool timeStop;

void printTextureData(GLuint tex, int width, int height) {
    // First, bind the texture to the current context
    glBindTexture(GL_TEXTURE_2D, tex);

    // Allocate a buffer to hold the texture data (assuming RGBA32F)
    int size = width * height;
    std::vector<cy::Vec4f> data(size);  // RGBA32F format (4 floats per texel)

    int happs = 0;
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_FLOAT, data.data());
    for (int i = 0; i < size; i++) {
        if ((data[i].x != 0 || data[i].y != 0 || data[i].z != 0 || data[i].w != 0))
            std::cout << "Readback texel " << i << ": (" << data[i].x << ", " << data[i].y << ", " << data[i].z << ", " << data[i].w << ")" << std::endl;
    }

    std::cout << "========================================\n" << std::endl;
}

static float* flattenMatrix4f(cy::Matrix4f mat) {
    float flat[16] = { 0 };

    for (int i = 0; i < 4; i++) {
        auto row = mat.GetRow(i);
        flat[i * 4 + 0] = row[0];
        flat[i * 4 + 1] = row[1];
        flat[i * 4 + 2] = row[2];
        flat[i * 4 + 3] = row[3];
    }

    return flat;
}

/// <summary>
/// Loads a compute shader from a file, compiles it, and links it into a program.
/// </summary>
GLuint loadComputeShader(const char* computeShaderPath) {
    std::ifstream file(computeShaderPath);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string source = buffer.str();
    const char* sourceCStr = source.c_str();

    GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(shader, 1, &sourceCStr, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "ERROR::COMPUTE_SHADER::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(shader);

    return program;
}

void runComputeShader() {
    // calculating delta time
    float t = glutGet(GLUT_ELAPSED_TIME);
    float dt = (t - oldT) / 1000.0f;
    oldT = t;

    glUseProgram(computeShaderProgram);

    // old physics snippet
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, flipFlop ? rigidbodyTexA : rigidbodyTexB);
    glUniform1i(glGetUniformLocation(computeShaderProgram, "rigidbodyTex"), 0);

    // set uniform vars
    glUniform1f(glGetUniformLocation(computeShaderProgram, "deltaTime"), timeStop ? 0 : dt);
    glUniform3f(glGetUniformLocation(computeShaderProgram, "gravity"), 0, GRAVITY, 0);
    glUniform1i(glGetUniformLocation(computeShaderProgram, "particleCount"), CUBE_PARTICLE_COUNT);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, particles);
    glUniform1i(glGetUniformLocation(computeShaderProgram, "rigidbodyParticles"), 1);

    // new physics snippet
    glBindImageTexture(0, flipFlop ? rigidbodyTexB : rigidbodyTexA, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);


    // dispatch
    glDispatchCompute(CUBES_TO_SPAWN, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    // flip-flop
    flipFlop = !flipFlop;
}

/// <summary>
/// Generates the particles in the scene
/// </summary>
void generateParticles() {
    glUseProgram(particleGenProgram);

    // bind rigidbody texture
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, flipFlop ? rigidbodyTexB : rigidbodyTexA);
    glUniform1i(glGetUniformLocation(particleGenProgram, "rigidbody"), 0);
    glUniform1i(glGetUniformLocation(particleGenProgram, "density"), CUBE_PARTICLE_DENSITY);


    // bind particle texture
    glBindImageTexture(0, particles, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA32F);


    // dispatch
    glDispatchCompute(CUBES_TO_SPAWN, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

/// <summary>
/// Assigns particles to a grid index
/// </summary>
void performGridGeneration() {
    glUseProgram(gridGenProgram);

    glBindTexture(GL_TEXTURE_2D, gridCountingStructure);
    glClearTexImage(gridCountingStructure, 0, GL_RGBA, GL_FLOAT, nullptr);
    glBindTexture(GL_TEXTURE_2D, gridDataStructure);
    glClearTexImage(gridDataStructure, 0, GL_RGBA, GL_FLOAT, nullptr);

    glUniform1i(glGetUniformLocation(gridGenProgram, "totalParticles"), CUBES_TO_SPAWN * CUBE_PARTICLE_COUNT);
    glUniform1f(glGetUniformLocation(gridGenProgram, "particleDiameter"), (CUBE_SCALE * 1.0f / CUBE_PARTICLE_DENSITY));
    glUniform1i(glGetUniformLocation(gridGenProgram, "particlesPerCube"), CUBE_PARTICLE_COUNT);
    glUniform1i(glGetUniformLocation(gridGenProgram, "particleDataSize"), PARTICLE_DATA_SIZE);
    glUniform1i(glGetUniformLocation(gridGenProgram, "gridSize"), WORLD_GRID_SIZE);

    glUniform3f(glGetUniformLocation(gridGenProgram, "smallestGridCoordinates"), -WORLD_GRID_SIZE / 2, -WORLD_GRID_SIZE / 2, -WORLD_GRID_SIZE / 2);

    glUniform1f(glGetUniformLocation(gridGenProgram, "springCoeff"), SPRING_COEFF);
    glUniform1f(glGetUniformLocation(gridGenProgram, "dampingCoeff"), DAMPING_COEFF);

    glBindImageTexture(0, particles, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    glBindImageTexture(1, gridDataStructure, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    glBindImageTexture(2, gridCountingStructure, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);


    glDispatchCompute(1, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

/// <summary>
/// Given the grid indices, calculate potential collisions
/// </summary>
void calculateParticleCollisions() {
    glUseProgram(particleCollisionProgram);

    glUniform1i(glGetUniformLocation(particleCollisionProgram, "totalParticles"), CUBES_TO_SPAWN * CUBE_PARTICLE_COUNT);
    glUniform1f(glGetUniformLocation(particleCollisionProgram, "particleDiameter"), CUBE_SCALE * 1.0f / CUBE_PARTICLE_DENSITY);
    glUniform1i(glGetUniformLocation(particleCollisionProgram, "particlesPerCube"), CUBE_PARTICLE_COUNT);
    glUniform1i(glGetUniformLocation(particleCollisionProgram, "particleDataSize"), PARTICLE_DATA_SIZE);
    glUniform1i(glGetUniformLocation(particleCollisionProgram, "gridSize"), WORLD_GRID_SIZE);

    glUniform3f(glGetUniformLocation(particleCollisionProgram, "smallestGridCoordinates"), -WORLD_GRID_SIZE / 2, -WORLD_GRID_SIZE / 2, -WORLD_GRID_SIZE / 2);

    glUniform1f(glGetUniformLocation(particleCollisionProgram, "springCoeff"), SPRING_COEFF);
    glUniform1f(glGetUniformLocation(particleCollisionProgram, "dampingCoeff"), DAMPING_COEFF);

    glBindImageTexture(0, particles, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    glBindImageTexture(1, gridDataStructure, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);
    glBindImageTexture(2, gridCountingStructure, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RGBA32F);


    glDispatchCompute(CUBES_TO_SPAWN * CUBE_PARTICLE_COUNT, 1, 1);
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

}

/// <summary>
/// Reads an object file and saves its data into the passed in VAO/VBOs:
/// 
/// </summary>
/// <param name="path">Path to the object file</param>
void loadObjFileAttributes(cy::TriMesh& mesh, const char* path, GLuint* vao, GLuint* vbo_pos, GLuint* vbo_norm, GLuint* vbo_tex, GLuint* ebo)
{
    bool success = mesh.LoadFromFileObj(path);

    // copy face data into a new array
    std::vector<cy::Vec3f> faces;
    std::vector<cy::Vec3f> normals;
    std::vector<cy::Vec2f> texture_coords;

    for (int i = 0; i < mesh.NF(); i++) {
        auto faceVerts = mesh.F(i).v;
        auto faceNormals = mesh.FN(i).v;
        auto faceText = mesh.FT(i).v;

        faces.push_back(mesh.V(faceVerts[0]));
        faces.push_back(mesh.V(faceVerts[1]));
        faces.push_back(mesh.V(faceVerts[2]));

        normals.push_back(mesh.VN(faceNormals[0]));
        normals.push_back(mesh.VN(faceNormals[1]));
        normals.push_back(mesh.VN(faceNormals[2]));

        texture_coords.push_back(cy::Vec2f(mesh.VT(faceText[0]).x, mesh.VT(faceText[0]).y));
        texture_coords.push_back(cy::Vec2f(mesh.VT(faceText[1]).x, mesh.VT(faceText[1]).y));
        texture_coords.push_back(cy::Vec2f(mesh.VT(faceText[2]).x, mesh.VT(faceText[2]).y));
    }

    // setup VAO
    glGenVertexArrays(1, vao);

    // positions
    glGenBuffers(1, vbo_pos);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo_pos);
    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(cy::Vec3f) * faces.size(),
        faces.data(),
        GL_STATIC_DRAW
    );

    // normals
    glGenBuffers(1, vbo_norm);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo_norm);
    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(cy::Vec3f) * normals.size(),
        normals.data(),
        GL_STATIC_DRAW
    );

    // textures
    glGenBuffers(1, vbo_tex);
    glBindBuffer(GL_ARRAY_BUFFER, *vbo_tex);
    glBufferData(
        GL_ARRAY_BUFFER,
        sizeof(cy::Vec2f) * texture_coords.size(),
        texture_coords.data(),
        GL_STATIC_DRAW
    );

    // element array buffer
    std::vector<GLuint> indices(faces.size());
    for (int i = 0; i < faces.size(); i++) { indices[i] = i; }

    glGenBuffers(1, ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(), indices.data(), GL_STATIC_DRAW);

}

void mouseInput(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON)
        LMB = (state == GLUT_DOWN);
    else if (button == GLUT_RIGHT_BUTTON)
        RMB = (state == GLUT_DOWN);

    lastMousePosX = x;
    lastMousePosY = y;
}

void mouseMotionInput(int x, int y) {
    int changeX = x - lastMousePosX;
    int changeY = y - lastMousePosY;

    if (LMB) {
        camYAngle += changeX * 0.01f;
        camXAngle += changeY * 0.01f;
    }
    if (RMB) {
        camDist -= changeY * 0.1f;
        camDist = std::max(1.0f, camDist);

        //std::cout << camDist << std::endl;
    }

    lastMousePosX = x;
    lastMousePosY = y;
}


void keyboardInput(unsigned char key, int x, int y) {
    if (key == 27)
        glutLeaveMainLoop();
    if (key == 13)
        timeStop = !timeStop;
}

void display() {
    // clearing 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ==================== CAMERA TRANSFORMATION ==================== //
    cy::Matrix4f rotationX, rotationY, translation, viewMatrix;
    rotationX.SetRotationX(camXAngle);
    rotationY.SetRotationY(camYAngle);
    translation.SetTranslation(cy::Vec3f(0.0f, 0.0f, -camDist));
    viewMatrix = translation * rotationY * rotationX;

    cy::Matrix4f projection;
    projection.SetPerspective(45.0f, 1.0f, 0.1f, ZFAR);

    cy::Matrix4f mvpMatrix = projection * viewMatrix;

    cy::Matrix4f normalMatrix = viewMatrix.GetTranspose().GetInverse();

    // ==================== DRAWING STUFF ==================== //

    // running compute shaders
    runComputeShader();
    generateParticles();
    performGridGeneration();
    calculateParticleCollisions();

    // drawing cubes
    prog.Bind();
    glBindVertexArray(cubeVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeEBO);

    prog["mvp"] = mvpMatrix;
    prog["normMat"] = normalMatrix;

    prog["lightPos"] = cy::Vec3f(25, 25, 25);
    prog["cameraPos"] = cy::Vec3f(0, 0, -camDist);
    prog["projectionMat"] = viewMatrix;

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, flipFlop ? rigidbodyTexA : rigidbodyTexB);
    prog["rigidbodyTex"] = 0;

    prog.EnableAttrib("pos");
    prog.SetAttribBuffer("pos", cubePositions, 3, GL_FLOAT, GL_FALSE);

    prog.EnableAttrib("norm");
    prog.SetAttribBuffer("norm", cubeNormals, 3, GL_FLOAT, GL_FALSE);

    glDrawElementsInstanced(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0, CUBES_TO_SPAWN);

    // swap buffers
    glutSwapBuffers();

}

void idle() {
    glutPostRedisplay();
}

// main method
int main(int argc, char** argv) {

    // ===== GLUT INITS ===== //
    glutInit(&argc, argv);
    glutInitContextVersion(4, 3);
    glutInitContextFlags(GLUT_DEBUG);


    // creating a window
    glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
    glutInitWindowPosition(0, 0);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow("Project 2 Window");

    //std::cout << "Setup done" << std::endl;

    // display function
    glutDisplayFunc(display);
    glutIdleFunc(idle);

    // gathering inputs
    glutKeyboardFunc(keyboardInput);
    glutMouseFunc(mouseInput);
    glutMotionFunc(mouseMotionInput);


    // ===== OPENGL INITS ===== //
    glClearColor(0, 0, 0, 1); // setting background color when clearing the screen
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);


    // ==================== PART 1 ==================== //
    // init glew
    glewInit();

    // for RNG
    srand(static_cast<unsigned int>(time(nullptr)));

    // setup initial physics stuff
    oldT = glutGet(GLUT_ELAPSED_TIME);

    // load program and mesh
    loadObjFileAttributes(cubeMesh, "../../objects/cube.obj", &cubeVAO, &cubePositions, &cubeNormals, &cubeTXCs, &cubeEBO);
    prog.BuildFiles("vertex.shader", "fragment.shader");

    // load particle data storage and rigidbody storage
    std::vector<cy::Vec4f> rbData(CUBES_TO_SPAWN * RIGIDBODY_DATA_SIZE);

    for (int i = 0; i < CUBES_TO_SPAWN; i++) {
        rbData[i * RIGIDBODY_DATA_SIZE + 0] = cy::Vec4f(
            ranrange(-50, 50),
            0,
            ranrange(-50, 50),
            1.0 / ranrange(1, 10)); // position, inv mass
        rbData[i * RIGIDBODY_DATA_SIZE + 1] = cy::Vec4f(
            ranrange(-INITIAL_MOMENTUM_X_RANGE, INITIAL_MOMENTUM_X_RANGE),
            ranrange(MIN_Y_INITIAL_MOMENTUM, MAX_Y_INITIAL_MOMENTUM),
            ranrange(-INITIAL_MOMENTUM_Z_RANGE, INITIAL_MOMENTUM_Z_RANGE),
            ranrange(MIN_OBJECT_SCALE, MAX_OBJECT_SCALE)); // linear momentum, object scale
        rbData[i * RIGIDBODY_DATA_SIZE + 2] = cy::Vec4f(
            ranrange(0, MAX_INITIAL_ANGULAR_MOMENTUM),
            ranrange(0, MAX_INITIAL_ANGULAR_MOMENTUM),
            ranrange(0, MAX_INITIAL_ANGULAR_MOMENTUM),
            0); // angular momenum, 0 (does nothing)
        rbData[i * RIGIDBODY_DATA_SIZE + 3] = randomQuaternion(); // orientation (identity)
        rbData[i * RIGIDBODY_DATA_SIZE + 4] = cy::Vec4f(
            ranrange(0, 1),
            ranrange(0, 1),
            ranrange(0, 1),
            1); // random color
    }

    // functions for loading initial data
    auto createRigidBodyTexture = [&](GLuint& tex) {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, RIGIDBODY_DATA_SIZE, CUBES_TO_SPAWN, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        };

    auto createParticlesTexture = [&](GLuint& tex) {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, PARTICLE_DATA_SIZE * CUBE_PARTICLE_COUNT, CUBES_TO_SPAWN, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        };

    auto createGridStructure = [&](GLuint& tex) {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WORLD_GRID_SIZE * WORLD_GRID_SIZE, WORLD_GRID_SIZE, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        };

    auto createGridCounter = [&](GLuint& tex) {
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, WORLD_GRID_SIZE * WORLD_GRID_SIZE, WORLD_GRID_SIZE, 0, GL_RGBA, GL_FLOAT, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        };

    // loading initial data
    createRigidBodyTexture(rigidbodyTexA);
    createRigidBodyTexture(rigidbodyTexB);
    createParticlesTexture(particles);
    createGridStructure(gridDataStructure);
    createGridCounter(gridCountingStructure);

    // upload initial data to the physics flip-flop textures
    glBindTexture(GL_TEXTURE_2D, rigidbodyTexA);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, RIGIDBODY_DATA_SIZE, CUBES_TO_SPAWN, GL_RGBA, GL_FLOAT, rbData.data());

    glBindTexture(GL_TEXTURE_2D, rigidbodyTexB);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, RIGIDBODY_DATA_SIZE, CUBES_TO_SPAWN, GL_RGBA, GL_FLOAT, rbData.data());

    // load the two compute shaders
    computeShaderProgram = loadComputeShader("compute.shader");
    particleGenProgram = loadComputeShader("particle_compute.shader");
    gridGenProgram = loadComputeShader("grid_generation_compute.shader");
    particleCollisionProgram = loadComputeShader("particle_collision_compute.shader");

    printf("%s\n", (const char*)glGetString(GL_VERSION));

    // call main loop
    glutMainLoop();
    return 0;
}

cy::Vec4f randomQuaternion() {
    float u1 = ranrange(0, 1);
    float u2 = ranrange(0, 1);
    float u3 = ranrange(0, 1);

    float sqrt1MinusU1 = sqrt(1.0f - u1);
    float sqrtU1 = sqrt(u1);

    float theta1 = 2.0f * PI * u2;
    float theta2 = 2.0f * PI * u3;

    float x = sqrt1MinusU1 * sin(theta1);
    float y = sqrt1MinusU1 * cos(theta1);
    float z = sqrtU1 * sin(theta2);
    float w = sqrtU1 * cos(theta2);

    return cy::Vec4f(x, y, z, w).GetNormalized();
}

// Function to generate a random float between min and max
float ranrange(float min, float max) {
    return min + (max - min) * (rand() / (RAND_MAX + 1.0f));
}