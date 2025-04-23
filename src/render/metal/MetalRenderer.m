// ==================================================================
// Implementation of Apple Metal Rendering of the Mesh Data
// ==================================================================

@import simd;
@import MetalKit;

#import "MetalRenderer.h"
#import "MetalMTKView.h"
#import "MetalShaderTypes.h"

#import "../meshdata.h"

// The one significant global variable
extern MeshData *mesh_data;

@implementation MetalRenderer {
  id<MTLDevice> _device;
  id<MTLRenderPipelineState> _pipelineState;
  id<MTLDepthStencilState> _depthStencilState;
  id<MTLCommandQueue> _commandQueue;
  vector_uint2 _viewportSize;
  id<MTLBuffer> _clothVertexBuffer;

  // Uniform
  id<MTLBuffer> mvpUniform;
  id<MTLBuffer> mUniform;
  id<MTLBuffer> vUniform;
  id<MTLBuffer> lightposition_Uniform;
  id<MTLBuffer> wireframeUniform;

  // The number of vertices in our vertex buffer;
  NSUInteger _numClothVertices;

  float translation_x;
  float translation_y;

  float zoom;

  float rotation_vertical;
  float rotation_horizontal;
}

- (nonnull instancetype)initWithMetalKitView:(nonnull MetalMTKView *)mtkView {
  _clothVertexBuffer = 0;
  mvpUniform = 0;
  mUniform = 0;
  vUniform = 0;
  lightposition_Uniform = 0;
  wireframeUniform = 0;

  self = [super init];
  if (self) {
    _device = mtkView.device;
    [self loadMetal:mtkView];
    [mtkView setRenderer:self];
    // initialize camera
    translation_x = 0;
    translation_y = 0;
    zoom = 0.3;
    rotation_vertical = 0;
    rotation_horizontal = 0;
  }

  return self;
}

NSMutableData *clothVertexData = NULL;

+ (nonnull NSData *)generateClothVertexData {
  NSUInteger dataSize = sizeof(float) * 12 * 3 * mesh_data->clothTriCount;
  [clothVertexData release];
  clothVertexData = [[NSMutableData alloc] initWithLength:dataSize];
  memcpy(clothVertexData.mutableBytes, mesh_data->clothTriData,
         sizeof(float) * 12 * 3 * mesh_data->clothTriCount);
  return clothVertexData;
}

- (void)loadMetal:(nonnull MTKView *)mtkView {
  mtkView.colorPixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;

  id<MTLLibrary> defaultLibrary = [_device newDefaultLibrary];

  id<MTLFunction> vertexFunction =
      [defaultLibrary newFunctionWithName:@"vertexShader"];
  id<MTLFunction> fragmentFunction =
      [defaultLibrary newFunctionWithName:@"fragmentShader"];

  MTLRenderPipelineDescriptor *pipelineStateDescriptor =
      [[MTLRenderPipelineDescriptor alloc] init];
  pipelineStateDescriptor.label = @"Simple Pipeline";
  pipelineStateDescriptor.vertexFunction = vertexFunction;
  pipelineStateDescriptor.fragmentFunction = fragmentFunction;
  pipelineStateDescriptor.colorAttachments[0].pixelFormat =
      MTLPixelFormatBGRA8Unorm;
  pipelineStateDescriptor.depthAttachmentPixelFormat =
      MTLPixelFormatDepth32Float;
  pipelineStateDescriptor.colorAttachments[0].pixelFormat =
      mtkView.colorPixelFormat;

  MTLDepthStencilDescriptor *depthStencilDescriptor =
      [MTLDepthStencilDescriptor new];
  depthStencilDescriptor.depthCompareFunction = MTLCompareFunctionLess;
  depthStencilDescriptor.depthWriteEnabled = YES;
  mtkView.depthStencilPixelFormat = MTLPixelFormatDepth32Float;
  _depthStencilState =
      [_device newDepthStencilStateWithDescriptor:depthStencilDescriptor];

  NSError *error = NULL;
  _pipelineState =
      [_device newRenderPipelineStateWithDescriptor:pipelineStateDescriptor
                                              error:&error];
  if (!_pipelineState) {
    NSLog(@"Failed to created pipeline state, error %@", error);
  }

  [self reGenerate];

  _commandQueue = [_device newCommandQueue];
}

- (void)cameraTranslate:(float)tx:(float)ty {
  translation_x += tx;
  translation_y += ty;
}

- (void)cameraRotate:(float)tx:(float)ty {
  rotation_vertical += 0.5 * ty;
  if (rotation_vertical > 89.99) {
    rotation_vertical = 89.99;
  }
  if (rotation_vertical < -89.99) {
    rotation_vertical = -89.99;
  }
  rotation_horizontal += 0.5 * tx;
}

- (void)cameraZoom:(float)tx:(float)ty {
  zoom *= pow(1.003, ty);
}

- (void)reGenerate {

  NSData *clothVertexData = [MetalRenderer generateClothVertexData];
  [_clothVertexBuffer release];
  _clothVertexBuffer =
      [_device newBufferWithLength:clothVertexData.length
                           options:MTLResourceStorageModeShared];
  memcpy(_clothVertexBuffer.contents, clothVertexData.bytes,
         clothVertexData.length);
  _numClothVertices = clothVertexData.length / sizeof(MetalVertex);
}

- (void)mtkView:(nonnull MTKView *)view drawableSizeWillChange:(CGSize)size {
  _viewportSize.x = size.width;
  _viewportSize.y = size.height;
}

- (matrix_float4x4)makeXRotationMatrix:(float)angle {
  matrix_float4x4 m = matrix_identity_float4x4;
  m.columns[1][1] = cos(angle);
  m.columns[2][1] = -sin(angle);
  m.columns[1][2] = sin(angle);
  m.columns[2][2] = cos(angle);
  return m;
}
- (matrix_float4x4)makeYRotationMatrix:(float)angle {
  matrix_float4x4 m = matrix_identity_float4x4;
  m.columns[0][0] = cos(angle);
  m.columns[2][0] = -sin(angle);
  m.columns[0][2] = sin(angle);
  m.columns[2][2] = cos(angle);
  return m;
}
- (matrix_float4x4)makeZRotationMatrix:(float)angle {
  matrix_float4x4 m = matrix_identity_float4x4;
  m.columns[0][0] = cos(angle);
  m.columns[1][0] = sin(angle);
  m.columns[0][1] = -sin(angle);
  m.columns[1][1] = cos(angle);
  return m;
}

- (matrix_float4x4)makeTranslationMatrix:(vector_float3)translation {
  matrix_float4x4 m = matrix_identity_float4x4;
  m.columns[3][0] = translation.x;
  m.columns[3][1] = translation.y;
  m.columns[3][2] = translation.z;
  return m;
}

- (matrix_float4x4)makeScaleMatrix:(float)xscale:(float)yscale:(float)zscale {
  matrix_float4x4 m = matrix_identity_float4x4;
  m.columns[0][0] = xscale;
  m.columns[1][1] = yscale;
  m.columns[2][2] = zscale;
  return m;
}

- (matrix_float4x4)makeOrthographicMatrix:(float)
                            field_of_view:(float)aspect
                                         :(float)near
                                         :(float)far {
  matrix_float4x4 m = matrix_identity_float4x4;

  float left = -0.65;
  float right = 0.65;
  float top = 0.65;
  float bottom = -0.65;

  m.columns[0][0] = 2.0 / (float)(right - left);
  m.columns[1][1] = 2.0 / (float)(top - bottom);
  m.columns[2][2] = -2.0 / (float)(far - near);

  m.columns[3][0] = -(float)(right + left) / (float)(right - left);
  m.columns[3][1] = -(float)(top + bottom) / (float)(top - bottom);
  m.columns[3][2] = (float)(far + near) / (float)(far - near);
  m.columns[3][3] = 1;

  return m;
}

- (matrix_float4x4)makePerspectiveMatrix:(float)
                           field_of_view:(float)aspect
                                        :(float)near
                                        :(float)far {
  matrix_float4x4 m = matrix_identity_float4x4;

  // work in progress

  float scale = 1.0 / tan(field_of_view * 0.5 * M_PI / 180.0);
  scale = 1;

  m.columns[0][0] = scale;
  m.columns[1][1] = scale;

  // m.columns[2][2] = 0;
  m.columns[2][2] = (far) / (far - near);
  // m.columns[2][3] = 0; //0.001;

  m.columns[3][2] = (far * near) / (far - near);
  m.columns[3][3] = 1;
  return m;
}

/// Called whenever the view needs to render a frame
- (void)drawInMTKView:(nonnull MTKView *)view {

  // Create a new command buffer for each render pass to the current drawable
  id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
  commandBuffer.label = @"MyCommand";

  // Obtain a renderPassDescriptor generated from the view's drawable textures
  MTLRenderPassDescriptor *renderPassDescriptor =
      view.currentRenderPassDescriptor;

  if (renderPassDescriptor != nil) {

    // set background color to white
    renderPassDescriptor.colorAttachments[0].clearColor =
        MTLClearColorMake(1.0, 1.0, 1.0, 1.0);

    // Create a render command encoder so we can render into something
    id<MTLRenderCommandEncoder> renderEncoder =
        [commandBuffer renderCommandEncoderWithDescriptor:renderPassDescriptor];
    renderEncoder.label = @"MyRenderEncoder";

    [renderEncoder setViewport:(MTLViewport){0.0, 0.0, _viewportSize.x,
                                             _viewportSize.y, -1.0, 1.0}];
    [renderEncoder setRenderPipelineState:_pipelineState];

    [renderEncoder setDepthStencilState:_depthStencilState];
    [renderEncoder setFrontFacingWinding:MTLWindingCounterClockwise];

    // cull neither (we want to see the blue "inside" / backside)
    //[renderEncoder setCullMode:MTLCullModeBack];
    //[renderEncoder setCullMode:MTLCullModeFront];

    // focus on the center of the boundingbox
    vector_float3 first_translation = {-mesh_data->bb_center.data[0],
                                       -mesh_data->bb_center.data[1],
                                       -mesh_data->bb_center.data[2]};
    vector_float3 first_scale = {mesh_data->bb_center.data[0],
                                 mesh_data->bb_center.data[1],
                                 mesh_data->bb_center.data[2]};

    vector_float3 translation = {-0.01 * translation_x, -0.01 * translation_y,
                                 0};
    float fov = 45;
    float aspect = 1;
    float near = -5;
    float far = 100;

    matrix_float4x4 modelMatrix =
        matrix_multiply([self makeScaleMatrix:mesh_data->
                                     bb_scale:mesh_data->
                                     bb_scale:mesh_data->bb_scale],
                        [self makeTranslationMatrix:first_translation]);
    matrix_float4x4 worldMatrix = matrix_multiply(
        [self makeXRotationMatrix:rotation_vertical * M_PI / 180],
        matrix_multiply(
            [self makeYRotationMatrix:rotation_horizontal * M_PI / 180],
            matrix_multiply([self makeScaleMatrix:zoom:zoom:zoom],
                            [self makeTranslationMatrix:translation])));

    matrix_float4x4 viewMatrix = matrix_identity_float4x4;

    matrix_float4x4 projectiveMatrix =
        [self makeOrthographicMatrix:fov:aspect:near:far];

    matrix_float4x4 modelViewProjectionTransformation = matrix_multiply(
        projectiveMatrix,
        matrix_multiply(viewMatrix, matrix_multiply(worldMatrix, modelMatrix)));

    vector_float3 lightPosition = {4, 4, 4};

    // Load the MVP transformation into the MTLBuffer
    if (mvpUniform == 0) {
      mvpUniform =
          [_device newBufferWithBytes:(void *)&modelViewProjectionTransformation
                               length:sizeof(modelViewProjectionTransformation)
                              options:MTLResourceOptionCPUCacheModeDefault];
      mUniform =
          [_device newBufferWithBytes:(void *)&modelMatrix
                               length:sizeof(modelMatrix)
                              options:MTLResourceOptionCPUCacheModeDefault];
      vUniform =
          [_device newBufferWithBytes:(void *)&viewMatrix
                               length:sizeof(viewMatrix)
                              options:MTLResourceOptionCPUCacheModeDefault];
      lightposition_Uniform =
          [_device newBufferWithBytes:(void *)&lightPosition
                               length:sizeof(lightPosition)
                              options:MTLResourceOptionCPUCacheModeDefault];
      wireframeUniform =
          [_device newBufferWithBytes:(void *)&mesh_data->wireframe
                               length:sizeof(int)
                              options:MTLResourceOptionCPUCacheModeDefault];
    } else {
      memcpy(mvpUniform.contents, &modelViewProjectionTransformation,
             sizeof(modelViewProjectionTransformation));
      memcpy(mUniform.contents, &modelViewProjectionTransformation,
             sizeof(modelMatrix));
      memcpy(vUniform.contents, &modelViewProjectionTransformation,
             sizeof(viewMatrix));
      memcpy(lightposition_Uniform.contents, &lightPosition,
             sizeof(lightPosition));
      memcpy(wireframeUniform.contents, &mesh_data->wireframe, sizeof(int));
    }

    [renderEncoder setVertexBytes:&_viewportSize
                           length:sizeof(_viewportSize)
                          atIndex:MetalVertexInputIndexViewportSize];
    [renderEncoder setVertexBuffer:mvpUniform offset:0 atIndex:2];
    [renderEncoder setVertexBuffer:mUniform offset:0 atIndex:3];
    [renderEncoder setVertexBuffer:vUniform offset:0 atIndex:4];
    [renderEncoder setVertexBuffer:lightposition_Uniform offset:0 atIndex:5];
    [renderEncoder setVertexBuffer:wireframeUniform offset:0 atIndex:6];

    // Send our data to the Metal vertex shader
    int numMeshVertices;
    numMeshVertices = mesh_data->clothTriCount * 3;
    if (numMeshVertices > 0) {
      [renderEncoder setVertexBuffer:_clothVertexBuffer
                              offset:0
                             atIndex:MetalVertexInputIndexVertices];
      [renderEncoder drawPrimitives:MTLPrimitiveTypeTriangle
                        vertexStart:0
                        vertexCount:numMeshVertices];
    }

    [renderEncoder endEncoding];
    [commandBuffer presentDrawable:view.currentDrawable];
  }

  // Finalize rendering here & push the command buffer to the GPU
  [commandBuffer commit];
}

@end
