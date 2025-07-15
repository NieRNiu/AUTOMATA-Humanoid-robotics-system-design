# Unitree RL Gym: G1 Quadruped Locomotion Experiments

This repository contains the code and configuration for training and evaluating a reinforcement-learning policy on the Unitree G1 quadruped using NVIDIA Isaac Gym and MuJoCo. We compare a **default** reward configuration against a **more flexible** variant to examine trade-offs between stability and agility.

---

## Table of Contents

* [Requirements](#requirements)
* [Installation & Setup](#installation--setup)
* [Training Experiments](#training-experiments)

  * [1. Default Reward Configuration](#1-default-reward-configuration)
  * [2. More Flexible Reward Configuration](#2-more-flexible-reward-configuration)
  * [3. Comparing Logs in TensorBoard](#3-comparing-logs-in-tensorboard)
* [Simulation & Deployment](#simulation--deployment)

  * [Isaac Gym Playback](#isaac-gym-playback)
  * [MuJoCo “Sim2Sim” Validation](#mujoco-sim2sim-validation)
* [Results & Analysis](#results--analysis)
* [Appendix](#appendix)

---

## Requirements

* Ubuntu 20.04
* NVIDIA GPU with CUDA support (for Isaac Gym)
* [Conda](https://docs.conda.io/)
* Python 3.8+
* MuJoCo 2.x (for deployment)

---

## Installation & Setup

1. **Clone the repository**

   ```bash
   git clone https://github.com/unitreerobotics/unitree_rl_gym.git
   cd unitree_rl_gym
   ```

2. **Create & activate the Conda environment**

   ```bash
   conda env create -f environment.yml
   conda activate rl-g1
   ```

3. **Install MuJoCo (for deployment)**
   Follow the [MuJoCo installation guide](https://mujoco.org/) and set `MUJOCO_PY_MJKEY_PATH`, `MUJOCO_PY_MJPRO_PATH` accordingly.

---

## Training Experiments

### 1. Default Reward Configuration

1. **Launch training (10 000 iterations)**

   ```bash
   python legged_gym/scripts/train.py \
     --task=g1 \
     --experiment_name=g1_default \
     --run_name=run1 \
     --headless
   ```

2. **Monitor training**

   ```bash
   tensorboard --logdir logs/g1_default
   ```

3. **Playback policy in Isaac Gym**

   ```bash
   python legged_gym/scripts/play.py \
     --task=g1 \
     --experiment_name=g1_default \
     --run_name=run1
   ```

4. **Export the trained policy**
   The network checkpoint is saved to:

   ```
   logs/g1_default/exported/policies/policy_lstm_1.pt
   ```

---

### 2. More Flexible Reward Configuration

We reduce penalties on base-height deviation and foot swing, and increase contact reward:

```python
class scales(LeggedRobotCfg.rewards.scales):
    tracking_lin_vel   = 1.0
    tracking_ang_vel   = 0.5
    lin_vel_z          = -2.0
    ang_vel_xy         = -0.05
    orientation        = -1.0
    base_height        = -3.0     # was -10.0
    dof_acc            = -2.5e-7
    dof_vel            = -1e-3
    feet_air_time      = 0.0
    collision          = 0.0
    action_rate        = -0.01
    dof_pos_limits     = -5.0
    alive              = 0.15
    hip_pos            = -1.0
    contact_no_vel     = -0.2
    feet_swing_height  = -5.0     # was -20.0
    contact            = 0.3      # was 0.18
```

1. **Train with flexible rewards (5 000 iterations)**

   ```bash
   python legged_gym/scripts/train.py \
     --task=g1 \
     --experiment_name=g1_flex \
     --run_name=g1_flex \
     --headless \
     --max_iteration=5000
   ```

2. **Monitor training**

   ```bash
   tensorboard --logdir logs/g1_flex
   ```

3. **Playback policy in Isaac Gym**

   ```bash
   python legged_gym/scripts/play.py \
     --task=g1 \
     --experiment_name=g1_flex \
     --run_name=g1_flex
   ```

4. **Export the policy**

   ```
   logs/g1_flex/exported/policies/policy_lstm_1.pt
   ```

---

### 3. Comparing Logs in TensorBoard

To directly compare default vs. flexible:

```bash
mkdir logs_compare
ln -s ../logs/g1_default   logs_compare/default
ln -s ../logs/g1_flex      logs_compare/flex
tensorboard --logdir logs_compare
```

---

## Simulation & Deployment

### Isaac Gym Playback

Run the `play.py` script as shown above to watch each policy in the Isaac Gym renderer.

### MuJoCo “Sim2Sim” Validation

1. **Edit the MuJoCo config**
   In `deploy/deploy_mujoco/configs/g1.yaml`, set:

   ```yaml
   policy_path: logs/g1_<default|flex>/exported/policies/policy_lstm_1.pt
   ```

2. **Launch MuJoCo deployment**

   ```bash
   python deploy/deploy_mujoco/deploy_mujoco.py g1.yaml
   ```

---

## Results & Analysis

* **Default (10 000 iter)**

  * Converges around 4 000 steps.
  * Stable forward velocity and contact rate \~0.35.
  * Gait is steady but somewhat stiff.

* **Flexible (5 000 iter)**

  * Faster rise in contact and base-height metrics.
  * More natural leg swing and smoother action profiles.
  * Slightly higher value-function loss variance (more exploration).

**Key take-aways:**

* Loosening height and swing penalties yields a livelier gait.
* Increasing contact reward encourages quicker, stronger footfalls.
* Additional velocity/acceleration penalties may be needed to tame variance.
* Both policies achieve stable walking in Isaac Gym & MuJoCo.

For full plots, see the **Appendix** in this document or browse TensorBoard under `logs/`.

---

## Appendix

All TensorBoard charts (loss, reward subterms, learning rate) are saved under:

* `logs/g1_default`
* `logs/g1_flex`

You can generate comparison figures with:

```bash
tensorboard --logdir logs_compare
```

---

> *This README summarizes the experimental setup, training commands, reward configurations, and high-level results for the Unitree G1 quadruped RL experiments.*
