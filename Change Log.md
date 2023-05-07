## Change log

# Spring-based trajectory generation and visualisation

- Added position vector for future positions and velocity deque for historical velocities in 'controller.h'.

- Implemented spring-based trajectory generation in 'Controller::update()'. Rotation vector currently not taken into account to predict future trajectory.

- 'Controller::setInputDirection' now takes into account historical velocities to provide for smoother transitions when keyboard inputs are suddenly changed.

- Updated 'ShadowApplication::drawTrajectory' function in 'application.cpp'.

- Gamepad controls currently not supported.


# Basic camera-based control and single step trajectory visualisation in 8 directions

- Right mouse button no longer has any strafing effect on the camera in 'camera.h'. Camera follows controller position and can still be rotated around controller position.

- 'Controller::update()' function now takes a 'TrackingCamera' object as an input.

- 'controller.update(camera)' function shifted to the 'ShadowApplication::draw()' function in 'application.cpp' in order to receive camera data.

- 'controller.direction' now determined by camera direction and keyboard inputs.

- Added 'vec3toV3D' and 'V3Dtovec3' functions within 'controller.cpp' for convenience.