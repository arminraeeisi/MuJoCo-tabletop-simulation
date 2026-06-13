from pathlib import Path
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_SCENE = PROJECT_ROOT / "third_party" / "franka_emika_panda" / "station_scene.xml"


class PandaDataCollectionEnv(gym.Env):
    """
    Minimal Gymnasium-style environment for the MuJoCo tabletop
    data-collection station.

    The goal is not to train a complete RL policy. The purpose is to show
    that the scene can be wrapped in a standard environment interface with
    reset() and step(action).
    """

    metadata = {"render_modes": []}

    def __init__(self, scene_path: str | Path = DEFAULT_SCENE, frame_skip: int = 5):
        super().__init__()

        self.scene_path = Path(scene_path)
        if not self.scene_path.exists():
            raise FileNotFoundError(
                f"Scene file not found: {self.scene_path}. "
                "Run `python scripts/prepare_scene.py` first."
            )

        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        self.frame_skip = frame_skip

        if self.model.nu > 0:
            ctrl_range = self.model.actuator_ctrlrange.copy()
            low = ctrl_range[:, 0]
            high = ctrl_range[:, 1]

            invalid_range = np.isclose(low, high)
            low[invalid_range] = -1.0
            high[invalid_range] = 1.0

            self.action_space = spaces.Box(
                low=low.astype(np.float32),
                high=high.astype(np.float32),
                dtype=np.float32,
            )
        else:
            self.action_space = spaces.Box(
                low=np.array([], dtype=np.float32),
                high=np.array([], dtype=np.float32),
                dtype=np.float32,
            )

        obs_size = self.model.nq + self.model.nv
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_size,),
            dtype=np.float32,
        )

    def _get_obs(self) -> np.ndarray:
        """
        Observation vector:
        - qpos: generalized positions
        - qvel: generalized velocities

        This is simple but valid for a minimal environment wrapper.
        """
        return np.concatenate([self.data.qpos, self.data.qvel]).astype(np.float32)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)

        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        else:
            mujoco.mj_resetData(self.model, self.data)

        mujoco.mj_forward(self.model, self.data)

        observation = self._get_obs()
        info = {
            "scene_path": str(self.scene_path),
            "nq": self.model.nq,
            "nv": self.model.nv,
            "nu": self.model.nu,
        }

        return observation, info

    def step(
        self,
        action: np.ndarray,
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)

        if self.model.nu > 0:
            action = np.clip(action, self.action_space.low, self.action_space.high)
            self.data.ctrl[:] = action

        for _ in range(self.frame_skip):
            mujoco.mj_step(self.model, self.data)

        observation = self._get_obs()

        reward = 0.0
        terminated = False
        truncated = False

        info = {
            "time": float(self.data.time),
        }

        return observation, reward, terminated, truncated, info


def main() -> None:
    env = PandaDataCollectionEnv()

    observation, info = env.reset()
    print("Environment reset successfully.")
    print("Observation shape:", observation.shape)
    print("Info:", info)

    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)

    print("One step executed successfully.")
    print("Action shape:", action.shape)
    print("Observation shape:", observation.shape)
    print("Reward:", reward)
    print("Terminated:", terminated)
    print("Truncated:", truncated)
    print("Info:", info)


if __name__ == "__main__":
    main()
