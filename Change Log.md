# Change log

- Right mouse button no longer has any strafing effect on the camera in 'camera.h'. Camera follows controller position and can still be rotated around controller position.

- 'Controller::update()' function now takes a 'TrackingCamera' object as an input.

- 'controller.update(camera)' function shifted to the 'ShadowApplication::draw()' function in 'application.cpp' in order to receive camera data.

- 'controller.direction' now determined by camera direction and keyboard inputs.

- Added 'vec3toV3D' and 'V3Dtovec3' functions within 'controller.cpp' for convenience.