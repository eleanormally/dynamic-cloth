// ==================================================================
// Apple Metal Rendering of the Mesh Data
// ==================================================================

@import MetalKit;
@class MetalMTKView;

@interface MetalRenderer : NSObject<MTKViewDelegate>
- (nonnull instancetype)initWithMetalKitView:(nonnull MetalMTKView *)mtkView;
- (void)reGenerate;
- (void)cameraTranslate :(float)tx :(float)ty;
- (void)cameraRotate :(float)tx :(float)ty;
- (void)cameraZoom :(float)tx :(float)ty;
@end

