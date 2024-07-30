## MPS Handtracking Usecase for MediaPipe

### Description

This notebook showcases one use case for MPS hand tracking to help with tracking
hands in camera space using popular open source hand tracker e.g. MediaPipe.
Given a RGB image, MediaPipe is able to detect hand landmarks in the image. Each
landmark also has a depth relative to the wrist landmark. For hand tracking
results to be useful, we need to figure out the pose in the camera space. MPS
provides hand wrists defined in the camera space, which will be useful to
pinpoint the MediaPipe HT results in the camera space. Please see the notebook
for more details.

Here is a preview of what the hand poses look like in the camera space in the
end:

![mediapipe_hands_in_world](mediapipe_hands_in_world.png)

### Requirements

The following libraries are needed to run the notebook.

```
pip install opencv-python
pip install mediapipe
pip install projectaria-tools'[all]'
```
