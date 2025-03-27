// ================================================================
// Parse the command line arguments and the input file
// ================================================================

#include <fstream>
#include <iostream>

#include "../args/argparser.h"
#include "../cloth/cloth.h"
#include "../render/meshdata.h"

#if __APPLE__
#include "../libs/matrix.h"
#else
#include <glm/gtc/type_ptr.hpp>
#endif

ArgParser* GLOBAL_args;

// The command line arguments
ArgParser::ArgParser(int argc, const char* argv[], MeshData* _mesh_data) {

  // set some default values
  mesh_data = _mesh_data;
  path = ".";
  cloth = NULL;

  // parse the command line arguments
  for (int i = 1; i < argc; i++) {
    if (argv[i] == std::string("--cloth")) {
      i++;
      assert(i < argc);
      separatePathAndFile(argv[i], path, cloth_file);
    } else if (argv[i] == std::string("--size")) {
      i++;
      assert(i < argc);
      mesh_data->width = atoi(argv[i]);
      i++;
      assert(i < argc);
      mesh_data->height = atoi(argv[i]);
    } else {
      std::cout << "ERROR: unknown command line argument " << i << ": '"
                << argv[i] << "'" << std::endl;
      exit(1);
    }
  }

  Load();
  GLOBAL_args = this;
  packMesh(mesh_data, cloth);
}

void ArgParser::Load() {
  delete cloth;
  if (cloth_file != "") {
    cloth = new Cloth(this);
  } else {
    cloth = NULL;
  }
}

void ArgParser::separatePathAndFile(const std::string& input, std::string& path,
                                    std::string& file) {
  // we need to separate the filename from the path
  // (we assume the vertex & fragment shaders are in the same directory)
  // first, locate the last '/' in the filename
  size_t last = std::string::npos;
  while (1) {
    int next = input.find('/', last + 1);
    if (next != (int)std::string::npos) {
      last = next;
      continue;
    }
    next = input.find('\\', last + 1);
    if (next != (int)std::string::npos) {
      last = next;
      continue;
    }
    break;
  }
  if (last == std::string::npos) {
    // if there is no directory in the filename
    file = input;
    path = ".";
  } else {
    // separate filename & path
    file = input.substr(last + 1, input.size() - last - 1);
    path = input.substr(0, last);
  }
}

void packMesh(MeshData* mesh_data, Cloth* cloth) {
  if (cloth != NULL)
    cloth->PackMesh();

  BoundingBox bbox;
  if (cloth != NULL) {
    bbox = cloth->getBoundingBox();
  }

  // the boundingbox center and size will be used to adjust the camera
  Vec3f center;
  bbox.getCenter(center);
  mesh_data->bb_center.data[0] = center.x();
  mesh_data->bb_center.data[1] = center.y();
  mesh_data->bb_center.data[2] = center.z();
  mesh_data->bb_max_dim = bbox.maxDim();
  mesh_data->bb_scale = 1.8 / float(bbox.maxDim());
}
