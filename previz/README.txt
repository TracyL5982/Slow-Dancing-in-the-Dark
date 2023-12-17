# Slow Dancing in the Dark

Youtube Link: https://youtu.be/MARaIu0d2ew

By Xinran (Tracy) Li

## Build Instructions
1. Go to the "previz" folder. `cd previz`
To run on MacOS, `cp Makefile.mac Makefile`.
To run on Windows, `cp Makefile.wsl Makefile`.

2. Call `make`

3. Run by calling `./previz` or `./previz startFrame endFrame`
`./previz` will render all the frames (frame 0 to frame 299).
`./previz startFrame endFrame` will render frame startFrame/6 to endFrame/6 (since we are going 6 frames at a time when rendering the animation)
Frame 0, frame 80, and frame 260 are my favorites! Run `./previz 0 6`, `./previz 480 486` `./previz 1560 1566` to render them.

4. To build the animation using the rendered PPMs, go to folder "frames" and run `./movieMaker`

Tips: Adjust windowWidth and windowHeight to change image resolution and rendering speed. The default image size is 640*480, this takes quite long time to render one image.

## Ray Tracing Effects
Implemented Depth of Field, Motion Blur, and Cook-Torrance reflectance.
