// ==================================================================
// OpenGL Rendering of the MeshData data
// ==================================================================

#include "OpenGLRenderer.h"
#include "../../libs/boundingbox.h"
#include "../../libs/matrix.h"
#include "../meshdata.h"
#include "OpenGLCamera.h"
#include "OpenGLCanvas.h"

// NOTE: These functions are also called by the Mac Metal Objective-C
// code, so we need this extern to allow C code to call C++ functions
// (without function name mangling confusion).

extern "C" {
void Animate();
}

// ====================================================================
OpenGLRenderer::OpenGLRenderer(MeshData* _mesh_data, ArgParser* args) {
  mesh_data = _mesh_data;

  OpenGLCanvas::initialize(args, mesh_data, this);

  // Initialize the MeshData
  setupVBOs();

  glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDisable(GL_CULL_FACE);

  // Create and compile our GLSL program from the shaders
  GLuint programID = LoadShaders(args->path + "/OpenGL.vertexshader",
                                 args->path + "/OpenGL.fragmentshader");

  // Get handles for our uniforms
  MatrixID = glGetUniformLocation(programID, "MVP");
  LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
  ViewMatrixID = glGetUniformLocation(programID, "V");
  ModelMatrixID = glGetUniformLocation(programID, "M");
  wireframeID = glGetUniformLocation(programID, "wireframe");

  while (!glfwWindowShouldClose(OpenGLCanvas::window)) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(programID);

    OpenGLCanvas::camera->glPlaceCamera();

    // transform the object as necessary to fit in the
    // (-1,-1,-1)->(1,1,1) box
    glm::vec3 bb_center(-mesh_data->bb_center.data[0],
                        -mesh_data->bb_center.data[1],
                        -mesh_data->bb_center.data[2]);
    glm::vec3 bb_scale(mesh_data->bb_scale, mesh_data->bb_scale,
                       mesh_data->bb_scale);
    glm::mat4 ModelMatrix(1.0);
    ModelMatrix = glm::scale<GLfloat>(ModelMatrix, bb_scale);
    ModelMatrix = glm::translate<GLfloat>(ModelMatrix, bb_center);

    // Build the matrix to position the camera based on keyboard and mouse input
    glm::mat4 ProjectionMatrix = OpenGLCanvas::camera->getProjectionMatrix();
    glm::mat4 ViewMatrix = OpenGLCanvas::camera->getViewMatrix();
    glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

    Animate();
    updateVBOs();

    // pass the matrix to the draw routines (for further editing)
    drawVBOs(MVP, ModelMatrix, ViewMatrix);

    // Swap buffers
    glfwSwapBuffers(OpenGLCanvas::window);
    glfwPollEvents();
  }

  cleanupVBOs();
  glDeleteProgram(programID);

  // Close OpenGL window and terminate GLFW
  glfwDestroyWindow(OpenGLCanvas::window);
  glfwTerminate();
  exit(EXIT_SUCCESS);
}

// ====================================================================

void OpenGLRenderer::setupVBOs() {
  HandleGLError("enter setupVBOs");
  glGenVertexArrays(1, &cloth_VaoId);
  glGenBuffers(1, &cloth_tris_VBO);
  HandleGLError("leaving setupVBOs");
}

void OpenGLRenderer::drawVBOs(const glm::mat4& mvp, const glm::mat4& m,
                              const glm::mat4& v) {
  HandleGLError("enter drawVBOs");
  Vec3f lightPos = Vec3f(4, 4, 4);
  glUniform3f(LightID, lightPos.x(), lightPos.y(), lightPos.z());
  glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);
  glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &m[0][0]);
  glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &v[0][0]);
  glUniform1i(wireframeID, mesh_data->wireframe);
  drawMesh();
  HandleGLError("leaving drawVBOs");
}

void OpenGLRenderer::cleanupVBOs() {
  HandleGLError("enter cleanupVBOs");
  cleanupMesh();
  HandleGLError("leaving cleanupVBOs");
}

// ====================================================================

void OpenGLRenderer::updateVBOs() {
  HandleGLError("enter updateVBOs");

  glBindVertexArray(cloth_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, cloth_tris_VBO);
  int sizeOfVertices = 3 * sizeof(glm::vec4) * mesh_data->clothTriCount * 3;
  glBufferData(GL_ARRAY_BUFFER, sizeOfVertices, mesh_data->clothTriData,
               GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)sizeof(glm::vec4));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)(sizeof(glm::vec4) * 2));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)sizeof(glm::vec4));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)(sizeof(glm::vec4) * 2));

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4), 0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)sizeof(glm::vec4));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 3 * sizeof(glm::vec4),
                        (void*)(sizeof(glm::vec4) * 2));

  HandleGLError("leaving updateVBOs");
}

void OpenGLRenderer::drawMesh() const {
  HandleGLError("in drawMesh");
  glBindVertexArray(cloth_VaoId);
  glBindBuffer(GL_ARRAY_BUFFER, cloth_tris_VBO);
  glDrawArrays(GL_TRIANGLES, 0, 3 * mesh_data->clothTriCount);
  glPointSize(2.0);
  HandleGLError("leaving drawMesh");
}

void OpenGLRenderer::cleanupMesh() {
  glDeleteBuffers(1, &cloth_tris_VBO);
  glDeleteBuffers(1, &cloth_VaoId);
}

// ====================================================================
