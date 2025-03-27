#include <string.h>
#include <cassert>
#include <fstream>
#include <iostream>
#include <random>

#include "../args/argparser.h"
#include "../cloth/cloth.h"
#include "../render/meshdata.h"

// ====================================================================

// default values for the MeshData variables
void INIT_MeshData(MeshData* mesh_data) {
  mesh_data->width = 400;
  mesh_data->height = 400;
  mesh_data->timestep = 0.01;
  mesh_data->animate = false;

  mesh_data->particles = true;
  mesh_data->surface = true;
  mesh_data->velocity = false;
  mesh_data->force = false;
  mesh_data->bounding_box = false;

  mesh_data->perspective = true;
  mesh_data->wireframe = true;
  mesh_data->gouraud = true;

  mesh_data->clothTriCount = 0;
  mesh_data->clothTriData = NULL;

  mesh_data->gravity.data[0] = 0;
  mesh_data->gravity.data[1] = -9.8;
  mesh_data->gravity.data[2] = 0;
}

// ====================================================================
// ====================================================================

// NOTE: These functions are called by the Objective-C code, so we
// need this extern to allow C code to call C++ functions (without
// function name mangling confusion).

// Also, they use global variables...

extern "C" {

void PackMesh() {
  packMesh(GLOBAL_args->mesh_data, GLOBAL_args->cloth);
}

void Step() {
  if (GLOBAL_args->cloth)
    GLOBAL_args->cloth->Animate();
}

void Animate() {
  if (GLOBAL_args->mesh_data->animate) {
    for (int i = 0; i < 10; i++) {
      Step();
    }
    PackMesh();
  }
}

void Load() {
  GLOBAL_args->Load();
}
}
