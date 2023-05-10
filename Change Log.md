# Change log
## Orientation trajectory (historical and future) visualisation

- Historical orientation visualisation implemented.

## Orientation trajectory visualisation

- Orientation trajectory visualisation implemented. `rot`, `rotHist`, `angVel`, `rotDesired`, `lambdaRot` attributes and `getRot` method added to `Controller` class. `rotHist` currently not used yet as historical orientation visualisation not yet implemented. Angles are defined off the z-axis and positive angles are anti-clockwise off the z-axis with the x-axis pointed up on the x-z plane.

- `Controller::setInputDirection` now updates `rotDesired`.

- `ShadowApplication::drawTrajectory` receives `rot` data via `getRot` method and renders the vectors accordingly.

- Added `angleBetweenVectors`, `minusPiToPi` and `angleToVector` methods to `mxm_utils.h` for convenience. Exercise caution when using `angleBetweenVectors` as it works well for vectors on the x-z plane but will likely give erroneous results if instead used on vectors that are coplanar with the y-basis vector.

- The `for (int x = 0; x < 3; x += 1)` loop in `Controller::update` has been amended to `for (int x = 0; x < 3; x += 2)` as the y-component plays no role in the trajectory currently and can be skipped.

## Trajectory (historical and future) visualisation

- `posHist` deque added to the `Controller` class.

- Historical trajectory visualisation implemented. `Controller::getPosHist` function returns a vector that ranges from size 0 to 4 by taking every n frames from the `posHist` deque. This vector is sent to `ShadowApplication::drawTrajectory` function for visualisation of historical trajectories. Currently, n is hard-coded to be 10 since having it vary based on real-time frame rate is very jittery. 

- `Controller::setInputDirection` no longer takes into account historical velocities. All this should be done via adjustment of the damping ratio `lambda` in `Controller.h`. `lambda` has been retuned for smooth results.

## Spring-based trajectory generation and visualisation

- Added position vector for future positions and velocity deque for historical velocities in `controller.h`.

- Implemented spring-based trajectory generation in `Controller::update`. Rotation vector currently not taken into account to predict future trajectory.

- `Controller::setInputDirection` now takes into account historical velocities to provide for smoother transitions when keyboard inputs are suddenly changed.

- Updated `ShadowApplication::drawTrajectory` function in `application.cpp`.

- Gamepad controls currently not supported.

## Basic camera-based control and single step trajectory visualisation in 8 directions

- Right mouse button no longer has any strafing effect on the camera in `camera.h`. Camera follows controller position and can still be rotated around controller position.

- `Controller::update` function now takes a `TrackingCamera` object as an input.

- `controller.update` function shifted to the `ShadowApplication::draw` function in `application.cpp` in order to receive camera data.

- `controller.direction` now determined by camera direction and keyboard inputs.

- Added `vec3toV3D` and `V3Dtovec3` functions within `controller.cpp` for convenience.
