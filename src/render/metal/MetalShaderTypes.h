// ==================================================================
// Shared data types between Metal Shaders and Objective C source code
// ==================================================================

#ifndef MetalShaderTypes_h
#define MetalShaderTypes_h

#include <simd/simd.h>

typedef enum MetalVertexInputIndex
{
    MetalVertexInputIndexVertices     = 0,
    MetalVertexInputIndexViewportSize = 1,
} MetalVertexInputIndex;

// position & color
typedef struct
{
  vector_float4 position;
  vector_float4 normal;
  vector_float4 color;
} MetalVertex;

/*
typedef struct
{
  matrix_float4x4 modelViewProjectionMatrix;
  matrix_float4x4 modelViewMatrix;
  matrix_float4x4 modelMatrix;

  vector_float3 lightPosition_worldspace;
  int wireframe;
  
  matrix_float3x3 normalMatrix;
} Uniforms;
*/

#endif
