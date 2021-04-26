# Inverted_Pendulum
Repo for different files regarding inverted pendulum project.

Eventual goal of this project is to train a Convolutional Neural Network (CNN), on simulated pendulum footage.

Then let the pendulum be stabilized by the CNN, as a proof that the setup is controllable and can work mechanically, I implemented LQR control first.

# Hardware (LQR control)
Components:
* 2x Arduino Nano
* 1x Cytron 10Amp 5-30 V DC Motor Driver
* 1x Variable Power Supply
* 1x 600P / R-encoder
* 1x 12v 251 RPM DC Motor (43.8:1)
* 2x 500mm, 12mm diameter steel rods
* 2x SC12UU slider bearings
* 1x HTD 3M pulley (24 teeth, bore diameter 8)
* 1x HTD 3M pully (24 teeth, bore diameter 14)
* 1x 1000mm HTD 3M timing belt
* 1x 150mm M8 threaded rod
* 1x 400mm M6 threaded rod
* Some 3d printed parts(STL's included)
* MPU6050 (optional, not currently used)
* 20 mA LED, 270 Ohm resistor, switch button
* Bunch of M3, M4 and M5 nuts and bolts
* Wooden plank of 600mm by 250mm

Animation of the model:
![Alt Text](Pictures/CAD_Animation.gif)

# Software
The pendulum will try and swing up between hard set bounds (140mm on each side). When it enters a controllable region the LQR control takes over (click on link for video):

[![Demo](https://img.youtube.com/vi/xbCsyzsChe0/0.jpg)](https://www.youtube.com/watch?v=xbCsyzsChe0)


#

