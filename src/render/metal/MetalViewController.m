// ==================================================================
// Apple Metal View Controller
// ==================================================================

@import AppKit;
#define PlatformViewController NSViewController
@import MetalKit;

#import "../meshdata.h"
#import "MetalMTKView.h"
#import "MetalRenderer.h"

@interface MetalViewController : PlatformViewController
@end

MetalRenderer *GLOBAL_renderer;

extern MeshData *mesh_data;

extern void Animate();
extern void PackMesh();
extern void Step();
extern void Load();

void myCFTimerCallback() {
  Animate();
  if (mesh_data->animate) {
    [GLOBAL_renderer reGenerate];
  }
}

@implementation MetalViewController {
  MetalMTKView *_view;
  MetalRenderer *_renderer;
}

- (void)viewDidLoad {
  [super viewDidLoad];

  // Set the view to use the default device
  _view = (MetalMTKView *)self.view;
  _view.device = MTLCreateSystemDefaultDevice();

  if (!_view.device) {
    NSLog(@"Metal is not supported on this device");
    return;
  }

  printf("allocate init with metal\n");
  _renderer = [[MetalRenderer alloc] initWithMetalKitView:_view];
  GLOBAL_renderer = _renderer;

  if (!_renderer) {
    NSLog(@"Renderer failed initialization");
    return;
  }

  // Initialize our renderer with the view size
  [_renderer mtkView:_view drawableSizeWillChange:_view.drawableSize];

  _view.delegate = _renderer;

  // setup timer for animation loop
  CFRunLoopRef runLoop = CFRunLoopGetCurrent();
  CFRunLoopTimerContext context = {0, self, NULL, NULL, NULL};
#define runrate 0.0001
  CFRunLoopTimerRef timer = CFRunLoopTimerCreate(
      kCFAllocatorDefault, 0.1, runrate, 0, 0, &myCFTimerCallback, &context);
  CFRunLoopAddTimer(runLoop, timer, kCFRunLoopCommonModes);
}

@end
