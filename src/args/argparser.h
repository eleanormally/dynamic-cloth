// ================================================================
// Parse the command line arguments and the input file
// ================================================================

#ifndef __ARG_PARSER_H__
#define __ARG_PARSER_H__

#include <random>
#include <string>

struct MeshData;
class Mesh;
class Cloth;
class BoundingBox;

// ====================================================================
// ====================================================================

class ArgParser {

 public:
  ArgParser(int argc, const char* argv[], MeshData* _mesh_data);

  double rand() {
    static std::random_device rd;
    static std::mt19937 engine(rd());
    static std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(engine);
  }

  // helper functions
  void separatePathAndFile(const std::string& input, std::string& path,
                           std::string& file);

  void Load();

  // ==============
  // REPRESENTATION
  // all public! (no accessors)

  std::string cloth_file;
  std::string path;

  Cloth* cloth;
  MeshData* mesh_data;
  BoundingBox* bbox;
};

extern ArgParser* GLOBAL_args;
void packMesh(MeshData* mesh_data, Cloth* cloth);

#endif
