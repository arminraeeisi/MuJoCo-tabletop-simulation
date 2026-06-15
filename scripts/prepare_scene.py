from pathlib import Path
import shutil
import subprocess
import sys
import re

PROJECT_ROOT = Path(__file__).resolve().parents[1]

ROBOT_BASE_HEIGHT = 0.08

EXTERNAL_DIR = PROJECT_ROOT / "external"
MENAGERIE_DIR = EXTERNAL_DIR / "mujoco_menagerie"

THIRD_PARTY_DIR = PROJECT_ROOT / "third_party"
PANDA_SOURCE_DIR = MENAGERIE_DIR / "franka_emika_panda"
PANDA_TARGET_DIR = THIRD_PARTY_DIR / "franka_emika_panda"

LICENSES_DIR = PROJECT_ROOT / "licenses"
CUSTOM_TRAY_PATH = PROJECT_ROOT / "scene" / "assets" / "custom_tray.stl"
BOTTLE_PATH = PROJECT_ROOT / "scene" / "assets" / "bottle.stl"


STATION_BLOCK = """
    <!-- ================= Tabletop data-collection station ================ -->

    <body name="robot_base_platform" pos="0 0 0.040">
      <geom name="robot_base_platform_geom"
            type="box"
            size="0.22 0.22 0.040"
            rgba="0.25 0.25 0.25 1"
            friction="1.0 0.005 0.0001"/>
    </body>

    <!-- Tabletop workspace. All dimensions are in meters. -->
    <body name="data_collection_table" pos="0.65 0 0.36">
      <geom name="tabletop"
            type="box"
            size="0.45 0.35 0.03"
            rgba="0.55 0.45 0.35 1"
            friction="1.0 0.005 0.0001"/>

      <geom name="table_leg_front_left"
            type="box"
            pos="0.35 0.25 -0.20"
            size="0.025 0.025 0.20"
            rgba="0.25 0.25 0.25 1"/>

      <geom name="table_leg_front_right"
            type="box"
            pos="0.35 -0.25 -0.20"
            size="0.025 0.025 0.20"
            rgba="0.25 0.25 0.25 1"/>

      <geom name="table_leg_back_left"
            type="box"
            pos="-0.35 0.25 -0.20"
            size="0.025 0.025 0.20"
            rgba="0.25 0.25 0.25 1"/>

      <geom name="table_leg_back_right"
            type="box"
            pos="-0.35 -0.25 -0.20"
            size="0.025 0.025 0.20"
            rgba="0.25 0.25 0.25 1"/>
    </body>

    <!-- Invisible primitive collision tray.
    The Fusion 360 STL tray is used for visual appearance.
    These box geoms provide stable physical contact. -->
    <body name="primitive_object_tray" pos="0.65 0 0.390">
      <!-- Tray base: 300 mm x 200 mm x 10 mm -->
      <geom name="tray_base"
            type="box"
            pos="0 0 0.005"
            size="0.150 0.100 0.005"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>

      <!-- Tray walls: approximately 40 mm high, 10 mm thick -->
      <geom name="tray_wall_left"
            type="box"
            pos="0 0.095 0.030"
            size="0.150 0.005 0.020"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>

      <geom name="tray_wall_right"
            type="box"
            pos="0 -0.095 0.030"
            size="0.150 0.005 0.020"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>

      <geom name="tray_wall_front"
            type="box"
            pos="0.145 0 0.030"
            size="0.005 0.090 0.020"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>

      <geom name="tray_wall_back"
            type="box"
            pos="-0.145 0 0.030"
            size="0.005 0.090 0.020"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>
    </body>

    <!-- Three free manipulation objects for interaction/data collection. -->
    <body name="red_cube" pos="0.58 -0.06 0.435">
      <freejoint/>
      <geom name="red_cube_geom"
            type="box"
            size="0.025 0.025 0.025"
            mass="0.05"
            rgba="0.9 0.1 0.1 1"
            friction="1.0 0.005 0.0001"/>
    </body>

    <body name="blue_cylinder" pos="0.68 0.04 0.450">
      <freejoint/>
      <geom name="blue_cylinder_geom"
            type="cylinder"
            size="0.025 0.04"
            mass="0.07"
            rgba="0.1 0.2 0.9 1"
            friction="1.0 0.005 0.0001"/>
    </body>

    <body name="green_cube" pos="0.73 -0.03 0.432">
      <freejoint/>
      <geom name="green_cube_geom"
            type="box"
            size="0.022 0.022 0.022"
            mass="0.04"
            rgba="0.1 0.8 0.2 1"
            friction="1.0 0.005 0.0001"/>
    </body>

    <!-- Overhead camera stand to represent a data-collection sensor setup. -->
    <body name="camera_stand" pos="0.86 -0.22 0.740">
      <!-- Vertical pole: bottom touches table surface at z = 0.39 -->
      <geom name="camera_stand_vertical"
            type="cylinder"
            size="0.015 0.35"
            rgba="0.05 0.05 0.05 1"/>

      <!-- Connector at the top of the pole -->
      <geom name="camera_stand_top_connector"
            type="box"
            pos="0 0 0.35"
            size="0.025 0.025 0.018"
            rgba="0.05 0.05 0.05 1"/>

      <!-- Diagonal arm from the pole toward the tray center -->
      <geom name="camera_stand_arm"
            type="box"
            pos="-0.105 0.11 0.35"
            size="0.17 0.012 0.012"
            euler="0 0 2.333"
            rgba="0.05 0.05 0.05 1"/>

      <!-- Camera body above the tray -->
      <geom name="camera_body"
            type="box"
            pos="-0.21 0.22 0.31"
            size="0.04 0.03 0.035"
            rgba="0.02 0.02 0.02 1"/>

      <!-- Downward-facing lens -->
      <geom name="camera_lens"
            type="cylinder"
            pos="-0.21 0.22 0.270"
            size="0.012 0.025"
            rgba="0.0 0.0 0.0 1"/>
    </body>

    <!-- Simple calibration board / visual reference target. -->
    <body name="calibration_board" pos="0.92 0.18 0.480">
      <geom name="calibration_board_panel"
            type="box"
            size="0.006 0.13 0.09"
            rgba="0.85 0.85 0.75 1"/>
      <geom name="calibration_board_marker_1"
            type="box"
            pos="-0.007 -0.06 0.04"
            size="0.002 0.02 0.02"
            rgba="0.05 0.05 0.05 1"/>
      <geom name="calibration_board_marker_2"
            type="box"
            pos="-0.007 0.00 0.00"
            size="0.002 0.02 0.02"
            rgba="0.05 0.05 0.05 1"/>
      <geom name="calibration_board_marker_3"
            type="box"
            pos="-0.007 0.06 -0.04"
            size="0.002 0.02 0.02"
            rgba="0.05 0.05 0.05 1"/>
    </body>
"""

BOTTLE_ASSET = """
    <!-- Fusion 360 authored bottle mesh.
         The STL is exported in millimeters, therefore scale is 0.001. -->
    <mesh name="fusion360_bottle_mesh"
          file="bottle.stl"
          scale="0.001 0.001 0.001"/>
"""

BOTTLE_BODY = """
    <body name="fusion360_bottle" pos="0.58 0.055 0.400">
      <freejoint/>

      <!-- Fusion 360 bottle mesh used for visual appearance only. -->
      <geom name="fusion360_bottle_visual"
            type="mesh"
            mesh="fusion360_bottle_mesh"
            rgba="0.10 0.35 0.90 1"
            contype="0"
            conaffinity="0"/>

      <!-- Simplified collision geometry for stable physics. -->
      <geom name="fusion360_bottle_collision_body"
            type="cylinder"
            pos="0 0 0.055"
            size="0.017 0.055"
            mass="0.06"
            rgba="0 0 0 0"
            friction="1.0 0.005 0.0001"/>
    </body>
"""

CUSTOM_TRAY_ASSET = """
    <!-- Fusion 360 authored tray mesh.
         The STL is exported in millimeters, therefore scale is 0.001. -->
    <mesh name="custom_tray_mesh"
          file="custom_tray.stl"
          scale="0.001 0.001 0.001"/>
"""


CUSTOM_TRAY_VISUAL_BODY = """
    <!-- Visual mesh authored in Fusion 360.
         Physics contacts are handled by invisible primitive tray geoms above. -->
    <body name="fusion360_tray_visual" pos="0.65 0 0.39">
      <geom name="fusion360_tray_visual_geom"
            type="mesh"
            mesh="custom_tray_mesh"
            rgba="0.04 0.04 0.04 1"
            contype="0"
            conaffinity="0"/>
    </body>
"""


def run_command(command: list[str], cwd: Path | None = None) -> None:
    print(f"Running: {' '.join(command)}")
    subprocess.run(command, cwd=cwd, check=True)


def clone_menagerie() -> None:
    EXTERNAL_DIR.mkdir(exist_ok=True)

    if MENAGERIE_DIR.exists():
        print("MuJoCo Menagerie already exists. Skipping clone.")
        return

    run_command(
        [
            "git",
            "clone",
            "--depth",
            "1",
            "https://github.com/google-deepmind/mujoco_menagerie.git",
            str(MENAGERIE_DIR),
        ]
    )


def copy_panda_model() -> None:
    if not PANDA_SOURCE_DIR.exists():
        raise FileNotFoundError(f"Could not find Panda source directory: {PANDA_SOURCE_DIR}")

    if PANDA_TARGET_DIR.exists():
        shutil.rmtree(PANDA_TARGET_DIR)

    shutil.copytree(PANDA_SOURCE_DIR, PANDA_TARGET_DIR)
    print(f"Copied Franka Panda model to: {PANDA_TARGET_DIR}")

def raise_panda_base() -> None:
    """
    Raise the Franka Panda base so it sits on top of the robot platform.
    """
    panda_xml_path = PANDA_TARGET_DIR / "panda.xml"

    if not panda_xml_path.exists():
        raise FileNotFoundError(f"Missing Panda XML file: {panda_xml_path}")

    xml = panda_xml_path.read_text()

    pattern = r'(<body\s+name="link0"\s+childclass="panda")(\s*>)'
    replacement = rf'\1 pos="0 0 {ROBOT_BASE_HEIGHT:.3f}"\2'

    xml, count = re.subn(pattern, replacement, xml, count=1)

    if count != 1:
        raise ValueError("Could not find Panda root body link0 in panda.xml.")

    panda_xml_path.write_text(xml)
    print(f"Raised Panda base by {ROBOT_BASE_HEIGHT:.3f} m.")

def create_station_scene() -> None:
    source_scene_path = PANDA_TARGET_DIR / "scene.xml"
    target_scene_path = PANDA_TARGET_DIR / "station_scene.xml"

    xml = source_scene_path.read_text()

    panda_assets_dir = PANDA_TARGET_DIR / "assets"
    panda_assets_dir.mkdir(parents=True, exist_ok=True)

    station_block = STATION_BLOCK

    if CUSTOM_TRAY_PATH.exists():
        shutil.copyfile(CUSTOM_TRAY_PATH, panda_assets_dir / "custom_tray.stl")

        if "</asset>" not in xml:
            raise RuntimeError("Could not find </asset> in Panda scene.xml.")

        xml = xml.replace("</asset>", CUSTOM_TRAY_ASSET + "\n  </asset>", 1)

        station_block += "\n" + CUSTOM_TRAY_VISUAL_BODY
        print("Custom Fusion 360 tray found. Adding visual mesh to scene.")

    else:
        print("No custom_tray.stl found yet. Creating scene with primitive tray only.")

    if BOTTLE_PATH.exists():
        shutil.copyfile(BOTTLE_PATH, panda_assets_dir / "bottle.stl")

        if "</asset>" not in xml:
            raise RuntimeError("Could not find </asset> in Panda scene.xml.")

        xml = xml.replace("</asset>", BOTTLE_ASSET + "\n  </asset>", 1)

        station_block += "\n" + BOTTLE_BODY
        print("Fusion 360 bottle found. Adding visual mesh to scene.")

    else:
        print("No bottle.stl found. Creating scene without Fusion 360 bottle.")

    if "</worldbody>" not in xml:
        raise RuntimeError("Could not find </worldbody> in Panda scene.xml.")

    xml = xml.replace("</worldbody>", station_block + "\n  </worldbody>", 1)

    target_scene_path.write_text(xml)
    print(f"Created station scene: {target_scene_path}")


def copy_licenses() -> None:
    LICENSES_DIR.mkdir(exist_ok=True)

    panda_license = PANDA_TARGET_DIR / "LICENSE"
    if panda_license.exists():
        shutil.copyfile(
            panda_license,
            LICENSES_DIR / "MUJOCO_MENAGERIE_FRANKA_PANDA_LICENSE.txt",
        )

    menagerie_license = MENAGERIE_DIR / "LICENSE"
    if menagerie_license.exists():
        shutil.copyfile(
            menagerie_license,
            LICENSES_DIR / "MUJOCO_MENAGERIE_TOP_LEVEL_LICENSE.txt",
        )

    print(f"Copied licenses to: {LICENSES_DIR}")


def main() -> None:
    try:
        clone_menagerie()
        copy_panda_model()
        raise_panda_base()
        create_station_scene()
        copy_licenses()
        print("\nScene preparation complete.")
        print("Next: run `python run_viewer.py` from the project root.")
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()