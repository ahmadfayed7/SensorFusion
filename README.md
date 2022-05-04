# Udacity Sensor Fusion Nanodegree Course

This contains my homework assignments and quiz solutions for the programming portions of this nanodegree. They will be partitioned into separate directories, one for each course. Each course will have separate build instructions. Please note that I have also included the YOLOv3 weights from the Camera course as part of this repo. You will need Git LFS to download the weights. Please visit <https://github.com/git-lfs/git-lfs/wiki/Installation> to install Git LFS. Once you do, clone the repo and in the repo directory please use the following to download the YOLOv3 weights.

```
$ cd <path to this repo>
$ git lfs fetch
```

The weights will appear in `SFND_Camera/detect_objects/dat/yolo/yolov3.weights`

Please note that the Radar section of this course uses MATLAB so you will need to have this software available to run the code for this part of the course.  This was not explicitly mentioned in the official overview of the course which alludes to only knowing C++.  However, MATLAB is only required for one part of the Radar section and it isn't explicitly required to complete the course.  Therefore, you can run most of the code in this section with Octave.  The final project can run in either MATLAB or Octave.

There is also a portion of the course that uses Python to give a brief introduction to Kalman Filter principles, but the knowledge required is very basic and does not use any external dependencies.  This also was not mentioned as part of the requirements in this course.  Any version of Python 3 should work.  The original code was originally in Python 2 which I have changed to Python 3 for longevity.
