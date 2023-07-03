<!-- Credits for the readme template: https://github.com/othneildrew/Best-README-Template/ -->
<a name="readme-top"></a>


<!-- PROJECT LOGO -->
<br />
<div align="center">


<h3 align="center">ROBOTICS PROJECT</h3>

  <p align="center">
    Final project for Introduction to Robotics course at the University of Trento.
    <br />
    <br /><br />
  </p>
</div>


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#presentation">Presentation</a></li>
    <li><a href="#installation and execution">Installation and Execution</a></li>
  </ol>
</details>
<br>


<!-- Presentation -->
## Presentation
You can find a complete report [here](https://github.com/Massiccio1/loco-update/blob/master/report.pdf) and the [vision](https://robotics-documentation.netlify.app) and [kinematics](https://kinematics-documentation.netlify.app/files.html) documentations.

Here are some videos with the tasks that the robot is able to solve:

Simulated environment:
* [video 1](https://drive.google.com/file/d/1jFcZW-3KpMie3RxVlrOiGXJyNLcw2CsK/view?usp=sharing)
* [video 2](https://drive.google.com/file/d/1paGnYKHQeFindFJ7TUa8cPtxs3kFMrAR/view?usp=sharing)
* [video 3](https://drive.google.com/file/d/1I5kACMZbk-l6aYJHNlmCPOlY_CSGLqlq/view?usp=sharing)

Real environmnet
* [video 1](https://drive.google.com/file/d/1wAoGDOVZ4I0NGGgRuazhKw2N9uytYSy-/view?usp=sharing)
* [video 2](https://drive.google.com/file/d/1CGS4adC-G6lzUFUpFVjinBqnWyfV6OMM/view?usp=sharing)
* [video 3](https://drive.google.com/file/d/1AizO1xYMepmObvASdX3puALV_6DuJdWM/view?usp=sharing)

<br>

## Installation and execution

It is suggested to follow the locosim readme until this point: https://github.com/mfocchi/locosim#python, for download and setup all the catkin - Ros workspace.

Go to ~/ros_ws/src

```
cd ~/ros_ws/src
```

and clone the repo

```
git clone https://github.com/Massiccio1/loco-update.git

```
for the simulation

```
git clone --branch sim https://github.com/Massiccio1/loco-update.git

```
go back to  ~/ros_ws , and run `catkin_make install`.
```
cd ..
```
```
catkin_make install
```

```
source devel/setup.bash
```

In another terminal run the ur5 robot launcher. <br>
Edit params.py to match simulation/real robot and soft gripper / 3 finger gripper. <br>

Run `ur5_generic.py` from inside loco-update
```
python3 ur5_generic.py
```

When the homing is finished
```
rosrun test_cpp test_cpp_node
```

and run visione_lab.py
```
python3 visione_lab.py
```

When the first vision message arrives, write "2" and type enter to start the procedure. <br>

If the compilation gives an error, in the folder `test_cpp/`, replace `CmakeLists.txt` with `CmakeLists.txt.bak`


<!-- Team members -->
## Team members

Damiano Bertolini - 217898 <br>
Matteo Beltrami - 217897 <br>
Massimo Girardelli - 209644


<p align="right">(<a href="#readme-top">back to top</a>)</p>


