# MuJoCo Simulation Scene Setup — Working Student Challenge

## Scene choice

I created a tabletop robotic data-collection station with a Franka Panda robot, a work table, a custom object tray, simple manipulation objects, an overhead camera stand, and a calibration board.

I chose this scene because it represents a realistic setup for collecting robotic manipulation and perception data. The robot can interact with objects in the tray, while the camera stand and calibration board represent supporting infrastructure for visual data collection, calibration, and sim-to-real alignment.

## Design decisions

The Franka Panda robot model is sourced from MuJoCo Menagerie. I used this model because it provides a high-quality pre-converted MJCF description with meshes, joints, actuators, and associated files.

For the station assets, I used a combination of MuJoCo primitive geometries and one authored CAD asset. The table, camera stand, manipulation objects, and calibration board are modeled with primitive MuJoCo geoms because they are simple, stable for physics simulation, easy to scale, and do not introduce unnecessary mesh or licensing complexity.

As a bonus asset, I modeled the object tray in Fusion 360 and exported it as an STL mesh. In the MuJoCo scene, the tray mesh is used as the visual representation, while simplified primitive box geometries are used as invisible collision geometry. This separates visual fidelity from contact stability and makes the simulation easier to debug.

All scene dimensions are in meters. Object poses, scales, and masses were chosen to be plausible for a tabletop manipulation setup.

## How to run

Tested on Ubuntu.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
python scripts/prepare_scene.py
python run_viewer.py