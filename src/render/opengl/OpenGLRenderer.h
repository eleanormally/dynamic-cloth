// ==================================================================
// OpenGL Rendering of the Mesh Data
// ==================================================================

#include "../../args/argparser.h"
#include "../meshdata.h"
#include "OpenGLCanvas.h"

class OpenGLRenderer {

 public:
  OpenGLRenderer(MeshData* _mesh_data, ArgParser* args);

  // HELPER FUNCTIONS FOR RENDERING
  void setupVBOs();
  void updateVBOs();
  void drawVBOs(const glm::mat4& MVP, const glm::mat4& M, const glm::mat4& V);
  void cleanupVBOs();

 private:
  // private helper functions for VBOs
  void setupMesh();
  void drawMesh() const;
  void cleanupMesh();

  // REPRESENTATION
  MeshData* mesh_data;

  GLuint cloth_tris_VBO;

  GLuint cloth_VaoId;

  GLuint MatrixID;
  GLuint LightID;
  GLuint ViewMatrixID;
  GLuint ModelMatrixID;
  GLuint wireframeID;
};
