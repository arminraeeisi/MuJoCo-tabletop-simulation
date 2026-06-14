# MuJoCo Simulation Scene Setup

## Short explanation

I chose a tabletop robotic data collection station because it represents a realistic setup for robot manipulation and perception tasks. The scene contains a Franka Panda robot positioned in front of a table, a tray with simple manipulation objects, an overhead camera stand, and a calibration board. The tray defines a repeatable workspace for object interaction, while the camera stand and calibration board represent the visual data collection and calibration infrastructure that would be useful for sim to real alignment.

The Franka Panda model is sourced from MuJoCo Menagerie. I modeled the object tray in Fusion 360 and exported it as an STL mesh to demonstrate a simple CAD to simulation asset pipeline. For stable physics, I used simplified invisible MuJoCo box geoms as collision geometry for the tray, while the STL mesh is used as the visual representation. The remaining station elements are modeled with MuJoCo primitive geoms because they are simple, robust, and easy to scale.

With more time, I would add calibrated camera parameters, image observations, randomized object poses, more realistic contact/material parameters, and a scripted data collection routine. I would also compare the simulated robot/object behavior with measurements from a real station to improve sim to real alignment.

## How to run

Tested on Ubuntu.

Create and activate a Python virtual environment:

python3 -m venv .venv

source .venv/bin/activate

Install dependencies:

pip install --upgrade pip

pip install -r requirements.txt

Prepare the scene:

python scripts/prepare_scene.py

Launch the MuJoCo viewer: 

python run_viewer.py

To test the minimal Gymnasium-style environment:

python env.py

## Interactivity

The Franka Panda can be moved manually through the MuJoCo viewer's Control panel. The actuator sliders change the corresponding robot joint positions, which can also be inspected in the Joint panel.

## Asset sources and licensing

Franka Panda MJCF model: MuJoCo Menagerie.

Custom object tray: authored in Fusion 360 and exported as STL.

Table, camera stand, calibration board, and simple manipulation objects: authored as MuJoCo primitive geoms.

Relevant open-source license files are included in the licenses folder.

## GitHub repository

https://github.com/arminraeeisi/MuJoCo-tabletop-simulation.git