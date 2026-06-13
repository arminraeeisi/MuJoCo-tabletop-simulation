from pathlib import Path
import time
import argparse

import mujoco
import mujoco.viewer


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_SCENE = PROJECT_ROOT / "third_party" / "franka_emika_panda" / "station_scene.xml"


def run_viewer(scene_path: Path) -> None:
    if not scene_path.exists():
        raise FileNotFoundError(
            f"Scene file not found: {scene_path}\n"
            "Run `python scripts/prepare_scene.py` first."
        )

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    print(f"Loaded scene: {scene_path}")
    print(f"Number of position coordinates nq: {model.nq}")
    print(f"Number of velocity coordinates nv: {model.nv}")
    print(f"Number of actuators nu: {model.nu}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Viewer launched.")
        print("Use the MuJoCo viewer interface to inspect the scene and move joints/controls.")

        while viewer.is_running():
            step_start = time.time()

            mujoco.mj_step(model, data)
            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = max(0.0, model.opt.timestep - elapsed)
            time.sleep(sleep_time)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        type=Path,
        default=DEFAULT_SCENE,
        help="Path to the MuJoCo MJCF scene XML.",
    )
    args = parser.parse_args()

    run_viewer(args.scene)


if __name__ == "__main__":
    main()
