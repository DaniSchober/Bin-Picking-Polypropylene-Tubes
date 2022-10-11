# Bin Picking of transparent medical grade polypropylene tubes

This project was proposed by Novo Nordisk (site Måløv), and carried out as a special course at the Technical University of Denmark. The planned system consists of two main parts: A tube picking and placing unit, and a tube labeling unit.

The project is a solution for autonomous random bin-picking of plastic centrifuge tubes of 50ml. A UR5e arm in combination with a Robotiq Hand-E Adaptive Gripper is used for the picking process. With small adjustments in the code, a usage of a suction or a vacuum gripper would also be possible.

## Getting Started
To run the bin-picking solution, the following requirements are needed:  

### Hardware Prerequisites
* a UR5e from Universal Robots
* a Zivid Two structured light camera
* a Robotiq Hand-E Adaptive Gripper
* a table to mount the robot and place the tubes

![Image](https://user-images.githubusercontent.com/75242605/195185081-233d701e-4854-48fb-bbd9-c5dbc46e0c29.jpeg)

### Installation
```python
pip install requirements.txt
```
Install the SDU RTDE interface.


## Usage
* Connect camera and UR5e to your computer and specify the IP address
* Perform the hand-eye-calibration as explained in robodk_hand_eye_calibration
* Run ```main.py```
* Select your desired operation as explained in the UI

## Contact
Daniel Schober - s212599@dtu.dk
