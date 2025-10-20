## Repo quick orientation

- Purpose: educational repository for state estimation, localization, and vehicle control exercises using Python and notebooks. Primary work lives under:
  - `State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/` — ES‑EKF implementation, data, and helper modules (`es_ekf.py`, `rotations.py`, `data/`).
  - `Vehicle_Control/Trajectory_tracking/` — controllers, simulation helpers and Carla integration snippets.

## Big-picture architecture and data flow

- This is not a packaged application but a collection of Jupyter notebooks + supportive Python modules used for assignments:
  1. Raw sensor-like data lives in `State_estimation_&_Localization/.../data/` (pickle files and small helper modules). Notebooks load `data/pt1_data.pkl` and related files.
 2. Notebooks (e.g., `es_ekf_1.ipynb`, `es_ekf_2.ipynb`, `es_ekf_3.ipynb`) orchestrate experiments: load data, set sensor variances, transform LIDAR into IMU frame, run ES‑EKF loop, and plot results.
 3. Helper modules implement math primitives and interfaces used by notebooks: `rotations.py` (Quaternion, axis-angle helpers), `es_ekf.py` (reference ES‑EKF implementation used in notebooks), and `data/utils.py`.

## Key files to reference when making changes

- `State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/es_ekf.py` — canonical algorithm code and likely unit-test target.
- `State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/rotations.py` — quaternion and rotation helper APIs used across notebooks. Prefer using these helpers rather than reimplementing math.
- `State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/data/` — loader and dataset utilities (`data.py`, `utils.py`).


## Local development and common workflows

- Environment: notebooks assume Python 3.x with numpy, matplotlib and Jupyter. Use the provided `requirements.txt` to set up dependencies:

  pip install -r requirements.txt

- Run notebooks in-place. On Windows (PowerShell) you can start Jupyter from repo root:

  jupyter notebook

- To run an ES‑EKF experiment non‑interactively, open `es_ekf.py` (or convert notebook cells into a script) and run with the same Python environment.

- To quickly validate your environment and data, run the minimal test script:

  python State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/test_es_ekf_short.py

## Project-specific conventions

- Notebooks contain working code and canonical parameter values (for example, `var_imu_f`, `var_imu_w`, `var_gnss`, `var_lidar` in `es_ekf_2.ipynb`). When changing filter behavior, update the notebook and the corresponding variables in `es_ekf.py` if present.
- Coordinate frames: LIDAR points are transformed into the IMU/vehicle frame using `C_li` and `t_i_li` inside notebooks prior to fusion. Keep this transformation consistent when adding sensors.
- Quaternion usage: use `Quaternion(*q)` and helper methods (`to_mat()`, `to_euler()`, `quat_mult_right()`, `quat_mult_left()`) defined in `rotations.py`. Notebooks rely on these helpers extensively.

## Patterns to follow in code edits

- Preserve notebook-first workflow: Prefer not to make changes in helper modules (`rotations.py`, `es_ekf.py`, `data/`). Instead use them as reference and update corresponding notebook cells to demonstrate behavior and plots.
- Small changes should include an example cell or short script demonstrating the change (a runnable snippet in a notebook cell or a small test script under the same folder).

## Integration points & external dependencies

- No external services; main integrations are with local data files under `data/` and optional CARLA usage referenced in `Vehicle_Control/Trajectory_tracking/` (look in README.md and `Carla commands.txt`).
- If adding CARLA workflows, document required CARLA version and how to launch its server in this repo's README.

## Debugging tips

- Common runtime issues are shape mismatches with sensor arrays and quaternion orientation misuse. Inspect shapes with `.shape` and print quaternion matrix `to_mat()` when something looks rotated incorrectly.
- For failing imports in notebooks, check relative paths — notebooks expect to be opened with the repo root as working directory so that `from rotations import ...` resolves.

## Tests & validation

- This repo contains no automated CI/tests. For quick validation, use the provided test script:

  python State_estimation_&_Localization/Vehicle_state_estimation_using_ES-EKF_and_Sensor_data/test_es_ekf_short.py

- When adding code, include a small verification cell or script that: loads `data/pt1_data.pkl`, runs a short ES‑EKF loop for a few timesteps, and asserts shapes and no exceptions.

## Examples (search/replace patterns you may need)

- To transform lidar points into IMU frame (example from `es_ekf_2.ipynb`):

  C_li = np.array([...])
  t_i_li = np.array([0.5, 0.1, 0.5])
  lidar.data = (C_li @ lidar.data.T).T + t_i_li

- Kalman update helper signature used in notebooks:

  measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check)

## When to ask for human help

- If you need dataset provenance (how pt1_data.pkl was generated) or CARLA version and launch steps, ask the repo owner; they are not discoverable from code alone.

---
If any of this is unclear or you want more detail about the ES‑EKF internals, tell me which files you'd like expanded examples for and I will update this file.
