# Leo Rover NERF Turret

## Overview

The project is basically what the title says: a NERF blaster turret attached to a Leo Rover.
It's cool and sort of works, you can see it action here: https://youtu.be/yBr_lUw0h70 or here: https://hackaday.io/project/189349-nerf-turret-mounted-on-a-leo-rover

As of right now the turret and the rover can be controlled with a joystick so I can blast anyone I want with it from safety.
Plus: A camera attached to the rover can be used to track humans. 
The goal is to develop this project further the moment I come up with ideas for the development.

## Why
Robots are cool, guns are cool -> robots with guns are even more fun. So that's why.

## Who

I work for Leo and my boss decided that I can try running a Youtube channel if i want to. I agreed. 
I haven't been told what am I supposed to be doing with it so I'm trying to get as much fun from it as humanly possible while still promoting Leo Rovers.

I'm a mechanical engineer and most of the time **I have no idea what I'm doing** when it comes to software.  
But following the idea I've been taught - banging rocks together until something works - I've created what you can see here.

**THATS WHY YOU SHOULDN'T ASSUME IT'S GOING TO WORK IF YOU USE ANY OF THE MATERIALS I PROVIDE**

I'm learning, but it's going to take time before I'll call my software "kinda good"/

## How to use it:

### Software 

Dynamixel SDK is necessary to run this project. Everything you'll need to learn how to get a hold on it can be sound here: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/

In it's current state It's also using another package I've created - used to control Leo Rover with mecanum wheels.
This one: https://github.com/fictionlab/leo_mecanum   ---- kacper@fictionlab.pl <- hit me up if i've forgotten to publish it

Build the project and run:

```bash
roslaunch nerf_turret turret_camera.launch
```

### Mechanics:


Parts list:
* Leo rover (can be skipped really. Or changed for any other mobile platform running ROS as long as you are willing to tinker with it)
* Nerf Perses (Used here because it can be easily converted into a remote-controlled doomsday weapon )
* 3D prints

I'm not happy enought about how it works and I'm 100% sure it's going to go through a few changes. That's why: that's it when it comes to the mechanical part of the instruction, for now. 

### Electronics:

In order to make the Nerf gun remote-controlled I've created a separate circuit that lets me control the motors through the use of 2 MOSFETs. 
In the future I'll try to provide some kind of schematics. 

##Links :
* https://www.youtube.com/channel/UClapAzA65GYj-H6woUBkp9Q
* https://github.com/LeoRover
* https://github.com/robotis-git
