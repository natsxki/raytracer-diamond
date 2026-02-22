// Main.cpp : final project (IGR Fundamentals of Computer Graphics)
//Lylia Mesa

#define _USE_MATH_DEFINES

#include "RigidSolver.hpp" 
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <fstream>
#include <string>

#include "Error.h"
#include "ShaderProgram.h"
#include "Camera.h"
#include "Mesh.h"


const std::string DEFAULT_MESH_FILENAME("../data/diamond.obj");

GLFWwindow *g_window = nullptr;
int g_windowWidth = 1024;
int g_windowHeight = 768;
std::shared_ptr<Camera> g_cam;

// Physics Globals
std::shared_ptr<RigidSolver> g_solver;
std::shared_ptr<BodyAttributes> g_physicsBody;
bool g_isSimulating = false; // Default: paused

bool g_mirrorMode = false; // Default: reflection mode
bool g_wallsEnabled = false; // Default: walls off
bool g_lightBackground = false; 
int g_controlMode = 0; // 0 = Camera, 1 = Light 1, 2 = Light 2
glm::vec3 g_light1Pos(-4.0f, 6.0f, 4.0f);
glm::vec3 g_light2Pos(4.0f, 6.0f, 4.0f);
glm::vec3 g_light1Color(1.0f, 0.2f, 0.2f); 
glm::vec3 g_light2Color(0.2f, 0.5f, 1.0f); 

// Camera Control
float g_meshScale = 1.0; 
bool g_rotatingP = false;
bool g_panningP = false;
bool g_zoomingP = false;
double g_baseX = 0.0, g_baseY = 0.0;
glm::vec3 g_baseTrans(0.0);
glm::vec3 g_baseRot(0.0);

// Ray tracing resources
GLuint g_quadVAO = 0;
GLuint g_quadVBO = 0;
GLuint g_tboTex = 0;     
GLuint g_tboBuffer = 0;  
int g_triangleCount = 0; 

void saveFrame(int width, int height) {
  std::vector<unsigned char> pixels(width * height * 3);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
  
  // Flip Y
  std::vector<unsigned char> flipped(width * height * 3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int src = (y * width + x) * 3;
      int dst = ((height - 1 - y) * width + x) * 3;
      flipped[dst] = pixels[src];
      flipped[dst+1] = pixels[src+1];
      flipped[dst+2] = pixels[src+2];
    }
  }
  static int frameCount = 0;
  std::string filename = "frame_" + std::to_string(frameCount++) + ".ppm";
  std::ofstream out(filename, std::ios::binary);
  out << "P6\n" << width << " " << height << "\n255\n";
  out.write(reinterpret_cast<char*>(flipped.data()), flipped.size());
  out.close();
  std::cout << "Saved " << filename << std::endl;
}

struct Scene {
  std::shared_ptr<Mesh> diamond = nullptr; 
  std::shared_ptr<ShaderProgram> rayTraceShader;
  
  glm::vec3 boxMin, boxMax;
  glm::vec3 scene_center = glm::vec3(0);
  float scene_radius = 1.f;

  void setupUniforms(int dispW, int dispH) {
    glm::mat4 view = g_cam->computeViewMatrix();
    glm::mat4 invView = glm::inverse(view);
    
    rayTraceShader->set("cameraPos", glm::vec3(invView[3]));
    rayTraceShader->set("cameraDir", -glm::vec3(invView[2]));
    rayTraceShader->set("cameraUp", glm::vec3(invView[1]));
    rayTraceShader->set("cameraRight", glm::vec3(invView[0]));
    
    rayTraceShader->set("fov", glm::radians(45.0f)); 
    rayTraceShader->set("aspect", (float)dispW / (float)dispH);
    rayTraceShader->set("resolution", glm::vec2(dispW, dispH));
    
    rayTraceShader->set("u_mirrorMode", g_mirrorMode ? 1 : 0);
    rayTraceShader->set("u_wallsEnabled", g_wallsEnabled ? 1 : 0);
    rayTraceShader->set("u_lightBackground", g_lightBackground ? 1 : 0);

    rayTraceShader->set("u_light1Pos", g_light1Pos);
    rayTraceShader->set("u_light2Pos", g_light2Pos);
    rayTraceShader->set("u_light1Color", g_light1Color);
    rayTraceShader->set("u_light2Color", g_light2Color);

    // physics matrix
    if (g_physicsBody) {
      glm::mat4 physicsModel = g_physicsBody->worldMat(); //
      rayTraceShader->set("u_model", physicsModel);
    } else {
      rayTraceShader->set("u_model", glm::mat4(1.0f));
    }

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_BUFFER, g_tboTex);
    rayTraceShader->set("sceneVertices", 0); 
    rayTraceShader->set("numTriangles", g_triangleCount); 
  }

  void drawQuad() {
    glBindVertexArray(g_quadVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
  }

  void render() {
    int displayW, displayH;
    glfwGetFramebufferSize(g_window, &displayW, &displayH);

    glViewport(0, 0, displayW, displayH);
    glClear(GL_COLOR_BUFFER_BIT);

    rayTraceShader->use();
    setupUniforms(displayW, displayH); 
    drawQuad(); 
    rayTraceShader->stop();
  }
};

Scene g_scene;

void initQuad() {
  float quadVertices[] = { 
    -1.0f,  1.0f, -1.0f, -1.0f,  1.0f, -1.0f, 
    -1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f  
  };
  glGenVertexArrays(1, &g_quadVAO);
  glGenBuffers(1, &g_quadVBO);
  glBindVertexArray(g_quadVAO);
  glBindBuffer(GL_ARRAY_BUFFER, g_quadVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
  glBindVertexArray(0);
}

void initMeshToTexture(std::shared_ptr<Mesh> mesh) {
  const auto& vertices = mesh->vertexPositions(); 
  const auto& triangles = mesh->triangleIndices(); 

  // Calculate center
  glm::vec3 center(0.0f);
  for (const auto& v : vertices) center += v.position;
  if (!vertices.empty()) center /= (float)vertices.size();

  // Calculate radius
  float maxDistSq = 0.0f;
  for (const auto& v : vertices) {
    glm::vec3 diff = v.position - center;
    float dSq = dot(diff, diff); 
    if(dSq > maxDistSq) maxDistSq = dSq;
  }
  float radius = sqrt(maxDistSq);
  if(radius < 1e-6) radius = 1.0f; // prevent division by zero

  std::cout << "[Init] Normalizing Mesh. Radius: " << radius << " -> 1.0" << std::endl;

  std::vector<float> bufferData;
  bufferData.reserve(triangles.size() * 3 * 3); 

  for (const auto& tri : triangles) {
    // Center and normalize 
    glm::vec3 v0 = (vertices[tri.x].position - center) / radius;
    glm::vec3 v1 = (vertices[tri.y].position - center) / radius;
    glm::vec3 v2 = (vertices[tri.z].position - center) / radius;

    bufferData.push_back(v0.x); bufferData.push_back(v0.y); bufferData.push_back(v0.z);
    bufferData.push_back(v1.x); bufferData.push_back(v1.y); bufferData.push_back(v1.z);
    bufferData.push_back(v2.x); bufferData.push_back(v2.y); bufferData.push_back(v2.z);
  }

  g_triangleCount = triangles.size(); 
  glGenBuffers(1, &g_tboBuffer);
  glBindBuffer(GL_TEXTURE_BUFFER, g_tboBuffer);
  glBufferData(GL_TEXTURE_BUFFER, bufferData.size() * sizeof(float), bufferData.data(), GL_STATIC_DRAW);

  glGenTextures(1, &g_tboTex);
  glBindTexture(GL_TEXTURE_BUFFER, g_tboTex);
  glTexBuffer(GL_TEXTURE_BUFFER, GL_RGB32F, g_tboBuffer); 
  
  g_meshScale = 1.0f;
}

void update(float currentTime) {
  if (g_isSimulating && g_solver) {
    float dt = 1.0f / 30.0f; //30 FPS step
    g_solver->step(dt);      
  }
}

void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  if(action == GLFW_PRESS) {
    if (key == GLFW_KEY_ESCAPE) {
      glfwSetWindowShouldClose(window, true);
    } 

    else if (key == GLFW_KEY_P) {
      g_isSimulating = !g_isSimulating;
      std::cout << (g_isSimulating ? "PLAYING" : "PAUSED") << std::endl;
    }
    else if (key == GLFW_KEY_M) {
      g_mirrorMode = !g_mirrorMode;
      std::cout << "Floor Mode: " << (g_mirrorMode ? "MIRROR" : "MATTE") << std::endl;
    }
    else if (key == GLFW_KEY_F) {
      g_cam->setPosition(glm::vec3(0.0, 2.0, 13.0));
      g_cam->setRotation(glm::vec3(-0.1, 0.0, 0.0));
      std::cout << "Camera reset"<< std::endl;
    }
    else if (key == GLFW_KEY_W) {
      g_wallsEnabled = !g_wallsEnabled;
      std::cout << "Walls: " << (g_wallsEnabled ? "ON" : "OFF") << std::endl;
    }
    else if (key == GLFW_KEY_L) {
      g_lightBackground = !g_lightBackground;
      std::cout << "Background: " << (g_lightBackground ? "LIGHT" : "DARK") << std::endl;
    }
    else if (key == GLFW_KEY_R) {
      g_isSimulating = false; 
      // diamond is recreated from scratch to guarantee null velocity, torque, momentum 
      g_physicsBody = std::make_shared<Cone>(1.0f, 2.0f, 2.0f); 
       
      // initial state
      g_physicsBody->X = Vec3f(0.0, 5.0, 0.0);      
      g_physicsBody->omega = Vec3f(0.5, 5.0, 0.0); 
      g_solver->init(g_physicsBody.get());
      std::cout << "RESET: New Diamond Created" << std::endl;
    }
    else if (key == GLFW_KEY_S) {
      int w, h;
      glfwGetFramebufferSize(window, &w, &h);
      saveFrame(w, h);
    }
    // ... inside the if(action == GLFW_PRESS) block, add these:
    else if (key == GLFW_KEY_C) {
      g_controlMode = 0;
      std::cout << "Controlling: CAMERA" << std::endl;
    } else if (key == GLFW_KEY_1) {
      g_controlMode = 1;
      std::cout << "Controlling: LIGHT 1 (Red)" << std::endl;
    } else if (key == GLFW_KEY_2) {
      g_controlMode = 2;
      std::cout << "Controlling: LIGHT 2 (Blue)" << std::endl;
    }
  }

  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    float moveSpeed = 0.5f; 
    glm::vec3 currentPos;
    
    if (g_controlMode == 0) currentPos = g_cam->getPosition();
    else if (g_controlMode == 1) currentPos = g_light1Pos;
    else if (g_controlMode == 2) currentPos = g_light2Pos;

    glm::vec3 newPos = currentPos;

    if (key == GLFW_KEY_UP) newPos += glm::vec3(0.0f, moveSpeed, 0.0f);
    else if (key == GLFW_KEY_DOWN) newPos += glm::vec3(0.0f, -moveSpeed, 0.0f);
    else if (key == GLFW_KEY_LEFT) newPos += glm::vec3(-moveSpeed, 0.0f, 0.0f);
    else if (key == GLFW_KEY_RIGHT) newPos += glm::vec3(moveSpeed, 0.0f, 0.0f);
    else if (key == GLFW_KEY_I) newPos += glm::vec3(0.0f, 0.0f, -moveSpeed);
    else if (key == GLFW_KEY_O) newPos += glm::vec3(0.0f, 0.0f, moveSpeed);


    if (g_controlMode == 0) g_cam->setPosition(newPos);
    else if (g_controlMode == 1) g_light1Pos = newPos;
    else if (g_controlMode == 2) g_light2Pos = newPos;
  
  }
}


void windowSizeCallback(GLFWwindow *window, int width, int height) {
  g_windowWidth = width;
  g_windowHeight = height;
  g_cam->setAspectRatio(static_cast<float>(width)/static_cast<float>(height));
  glViewport(0, 0, (GLint)width, (GLint)height);
}

void cursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  const float normalizer = static_cast<float>((width + height)/2);
  const float dx = static_cast<float>((g_baseX - xpos) / normalizer);
  const float dy = static_cast<float>((ypos - g_baseY) / normalizer);
  if(g_rotatingP) {
    const glm::vec3 dRot(-dy*M_PI, dx*M_PI, 0.0);
    g_cam->setRotation(g_baseRot + dRot);
  } else if(g_panningP) {
    g_cam->setPosition(g_baseTrans + g_meshScale*glm::vec3(dx, dy, 0.0));
  } else if(g_zoomingP) {
    g_cam->setPosition(g_baseTrans + g_meshScale*glm::vec3(0.0, 0.0, dy));
  }
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
  if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
    if(!g_rotatingP) {
      g_rotatingP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseRot = g_cam->getRotation();
    }
  } else if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
    g_rotatingP = false;
  } else if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
    if(!g_panningP) {
      g_panningP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseTrans = g_cam->getPosition();
    }
  } else if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
    g_panningP = false;
  } else if(button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
    if(!g_zoomingP) {
      g_zoomingP = true;
      glfwGetCursorPos(window, &g_baseX, &g_baseY);
      g_baseTrans = g_cam->getPosition();
    }
  } else if(button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
    g_zoomingP = false;
  }
  
}

void exitOnCriticalError(const std::string &message) {
  std::cerr << "> [Critical error] " << message << std::endl;
  std::exit(EXIT_FAILURE);
}

void initOpenGL() {
  if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    exitOnCriticalError("[Failed to initialize OpenGL context]");
  glDisable(GL_DEPTH_TEST); 
  glDisable(GL_CULL_FACE);  
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f); 

  try {
    g_scene.rayTraceShader = ShaderProgram::genBasicShaderProgram("../src/raytracer.vert", "../src/raytracer.frag");
  } catch(std::exception &e) {
    exitOnCriticalError(std::string("[Error loading shader] ") + e.what());
  }
}

void initScene(const std::string &meshFilename) {
  int width, height;
  glfwGetWindowSize(g_window, &width, &height);
  g_cam = std::make_shared<Camera>();
  g_cam->setAspectRatio(static_cast<float>(width)/static_cast<float>(height));

  g_scene.diamond = std::make_shared<Mesh>();
  try {
    Mesh::loadOBJ(meshFilename, g_scene.diamond); 
  } catch(std::exception &e) {
    exitOnCriticalError(std::string("[Error loading mesh] ") + e.what());
  }
  initQuad();
  
  // noramlized diamond
  initMeshToTexture(g_scene.diamond);
  
  // update shader uniforms about size
  g_scene.scene_center = glm::vec3(0.0f);
  g_scene.scene_radius = 1.0f;
  g_meshScale = 1.0f;

  // Physics set up : Box size 1.0 for noramlized mesh 
  g_physicsBody = std::make_shared<Cone>(1.0f, 2.0f, 2.0f); 
  
  g_physicsBody->X = Vec3f(0.0, 4.0, 0.0);      
  g_physicsBody->omega = Vec3f(0.5, 3.0, 0.0);  
  
  g_solver = std::make_shared<RigidSolver>(g_physicsBody.get(), Vec3f(0, -9.8, 0));
  g_solver->init(g_physicsBody.get());
  
  g_isSimulating = false; 

  // Camera set up 
  g_cam->setPosition(glm::vec3(0.0, 2.0, 13.0));
  g_cam->setRotation(glm::vec3(-0.1, 0.0, 0.0)); 
}

void init(const std::string &meshFilename) {
  if(!glfwInit()) std::exit(EXIT_FAILURE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); 
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

  g_window = glfwCreateWindow(g_windowWidth, g_windowHeight, "RayTracer", nullptr, nullptr);
  if(!g_window) { glfwTerminate(); std::exit(EXIT_FAILURE); }
  
  glfwMakeContextCurrent(g_window);
  glfwSetWindowSizeCallback(g_window, windowSizeCallback);
  glfwSetKeyCallback(g_window, keyCallback);
  glfwSetCursorPosCallback(g_window, cursorPosCallback);
  glfwSetMouseButtonCallback(g_window, mouseButtonCallback);

  initOpenGL();                 
  initScene(meshFilename);      
}

void clear() {
  g_cam.reset();
  g_scene.diamond.reset();
  g_scene.rayTraceShader.reset();
  g_solver.reset();
  g_physicsBody.reset();
  glfwDestroyWindow(g_window);
  glfwTerminate();
}

int main(int argc, char **argv) {
  init(argc==1 ? DEFAULT_MESH_FILENAME : argv[1]);
  while(!glfwWindowShouldClose(g_window)) {
    update(static_cast<float>(glfwGetTime())); 
    g_scene.render();
    glfwSwapBuffers(g_window);
    glfwPollEvents();
  }
  clear();
  return EXIT_SUCCESS;
}