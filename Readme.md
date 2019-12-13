# Competition 5

This repository contains our code for the fifth competition of [CMPUT 412, Experimental Mobile Robotics](https://www.ualberta.ca/computing-science/undergraduate-studies/course-directory/courses/experimental-mobile-robotics).
The challenges in this competition are identical to the previous competition, [competition 4](https://github.com/CMPUT412-2019/competition4), our solution to which builds on our solutions to competitions [3](https://github.com/cmput412-2019/competition3) and [2](https://github.com/cmput412-2019/cmput412-competition2).

As [described previously](https://github.com/CMPUT412-2019/competition4#purpose), the competition consists of a course on which the robot must demonstrate various tasks, such as line following, shape detection, and AR tag detection.
This competition differs only in the scoring: multiple runs around the track are allowed, and points are cumulative in all runs.
The point amounts are also re-balanced to favour box-pushing.
For details on the individual tasks, see [our previous report](https://github.com/CMPUT412-2019/competition4#competition-overview).

# Setup and building

The project requires ROS Kinetic with the turtlebot, openni, ar_track_alvar, usb_cam, amcl and gmapping packages. Additionally, gTTs is required to generate the sounds.

    pip install gTTS

The robot is setup as the following, with an RGB-D camera facing forward and a webcam on the bottom front, looking down:

| Robot setup | RGB-D camera | Bottom camera
:-:|:-:|:-:
![](images/turtlebot.jpg) | ![](images/main_camera.jpg) | ![](images/bottom_camera.jpg)

Two blocks of styrofoam are taped to either side of the robot in order to aid box-pushing while not blocking the view of the bottom camera:

| Styrofoam setup (front view) | Styrofoam setup (top view)
:-:|:-:
![](images/styrofoam_front.jpg) | ![](images/styrofoam_top.jpg)

The code should be procured by cloning this repository. To build the components ROS needs to run the code, use

    cd competition5
    catkin build
    source devel/setup.bash

Then, generate the sound files

    cd util/sound
    python gen.py
    cd -

Finally, obtain `fastai_trained.onnx` from the releases of this repository and copy it to `src/feature_detector/scripts/server`.

# Running the code

Plug the Kobuki base, both cameras and a joystick into your laptop. Then run the launch file

    roslaunch competition4 competition4.launch

and run the python file `src/competition4/scripts/competition4.py`.

# Method

