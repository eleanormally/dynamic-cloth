// ==================================================================
// We need to derive our own MTKView to capture mouse & keyboard input
// ==================================================================

#import "MetalMTKView.h"
#import "../meshdata.h"
#import "MetalKeys.h"
#import "MetalRenderer.h"

extern MeshData *mesh_data;

// =======================
// state variables
// =======================
@implementation MetalMTKView {
  bool shift_pressed;
  bool control_pressed;
  bool option_pressed;
  bool command_pressed;
  float mouse_x;
  float mouse_y;
  MetalRenderer *renderer;
}

- (void)setRenderer:(MetalRenderer *)r {
  renderer = r;
}

- (BOOL)acceptsFirstResponder {
  return YES;
}

// =======================
// modifier keys
// =======================
- (void)flagsChanged:(NSEvent *)event {
  shift_pressed = [event modifierFlags] & NSEventModifierFlagShift;
  control_pressed = [event modifierFlags] & NSEventModifierFlagControl;
  option_pressed = [event modifierFlags] & NSEventModifierFlagOption;
  command_pressed = [event modifierFlags] & NSEventModifierFlagCommand;
}

extern void Animate();
extern void PackMesh();
extern void Step();
extern void Load();

// =======================
// regular keys
// =======================
- (void)keyDown:(NSEvent *)event {
  switch (event.keyCode) {
  case (KEY_A): {
    // start continuous animation
    if (!mesh_data->animate) {
      printf("animation started, press 'X' to stop\n");
      mesh_data->animate = true;
    }
    break;
  }
  case (KEY_X): {
    // stop continuous animation
    if (mesh_data->animate) {
      printf("animation stopped, press 'A' to start\n");
      mesh_data->animate = false;
    }
    break;
  }
  case (KEY_SPACE): {
    // a single step of animation
    mesh_data->animate = false;
    printf("press 'A' to start continuous animation\n");
    Step();
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_M): {
    mesh_data->particles = !mesh_data->particles;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_V): {
    mesh_data->velocity = !mesh_data->velocity;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_F): {
    mesh_data->force = !mesh_data->force;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_S): {
    mesh_data->surface = !mesh_data->surface;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_W): {
    mesh_data->wireframe = !mesh_data->wireframe;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_G): {
    mesh_data->gouraud = !mesh_data->gouraud;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_B): {
    mesh_data->bounding_box = !mesh_data->bounding_box;
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_R): {
    Load();
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_EQUALS): {
    mesh_data->timestep *= 2.0;
    printf("timestep doubled: %f\n", mesh_data->timestep);
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_MINUS): {
    mesh_data->timestep /= 2.0;
    printf("timestep halved: %f\n", mesh_data->timestep);
    PackMesh();
    [renderer reGenerate];
    break;
  }
  case (KEY_Q): {
    printf("quit\n");
    exit(0);
  }
  default: {
    printf("UNKNOWN key down %d\n", event.keyCode);
    break;
  }
  }
}

- (void)keyUp:(NSEvent *)event {
  switch (event.keyCode) {
  default: { /*printf ("UNKNOWN key up %d\n", event.keyCode); */
    break;
  }
  }
}

// =======================
// mouse actions
// =======================
- (void)mouseDown:(NSEvent *)event {
  NSPoint touchPoint = [event locationInWindow];
  int which = event.buttonNumber;
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
}
- (void)rightMouseDown:(NSEvent *)event {
  [self mouseDown:event];
}
- (void)otherMouseDown:(NSEvent *)event {
  [self mouseDown:event];
}

- (void)mouseDragged:(NSEvent *)event {
  NSPoint touchPoint = [event locationInWindow];
  float delta_x = mouse_x - touchPoint.x;
  float delta_y = mouse_y - touchPoint.y;
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
  int which = event.buttonNumber;
  // to fake other mouse buttons...
  // if (shift_pressed) which = 0;
  if (control_pressed)
    which = 0;
  if (option_pressed)
    which = 2;
  if (command_pressed)
    which = 1;
  if (which == 0) {
    [renderer cameraRotate:delta_x:delta_y];
  } else if (which == 2) {
    [renderer cameraZoom:delta_x:delta_y];
  } else {
    assert(which == 1);
    [renderer cameraTranslate:delta_x:delta_y];
  }
}
- (void)rightMouseDragged:(NSEvent *)event {
  [self mouseDragged:event];
}
- (void)otherMouseDragged:(NSEvent *)event {
  [self mouseDragged:event];
}

- (void)mouseUp:(NSEvent *)event {
  NSPoint touchPoint = [event locationInWindow];
  mouse_x = touchPoint.x;
  mouse_y = touchPoint.y;
}
- (void)rightMouseUp:(NSEvent *)event {
  [self mouseUp:event];
}
- (void)otherMouseUp:(NSEvent *)event {
  [self mouseUp:event];
}

@end
