# Installation

**Environment used**
- GPU: RTX 3090 24 GB × 2
- NVIDIA driver: 535.216.03
- CUDA: 12.1
- cuDNN: 8.9.2

## 1. Get the repository

Clone it from GitHub. All commands below run from the repository root `VLA_MIRE/`.

```bash
git clone --recursive https://github.com/yurangja99/VLA_MIRE.git VLA_MIRE
cd VLA_MIRE
```

## 2. Conda environment (`openvla-oft`)

```bash
conda create -n openvla-oft python=3.10 -y
conda activate openvla-oft

# PyTorch 2.2.0 (CUDA 12.1 wheels)
pip install torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 --index-url https://download.pytorch.org/whl/cu121

# OpenVLA-OFT (installs the OFT fork of transformers along with it)
cd openvla-oft
pip install -e .
pip install packaging ninja
pip install "flash-attn==2.5.5" --no-build-isolation

# LIBERO: the top-level libero/ has no __init__.py, so it needs a compat-mode editable install
cd ..
pip install -e LIBERO --config-settings editable_mode=compat
pip install -r openvla-oft/experiments/robot/libero/libero_requirements.txt

# Pinned versions (other versions fail or change the results)
pip install mujoco==3.2.3 numpy==1.26.4 "tensorflow-metadata==1.14.0" "protobuf==3.20.3" \
  wandb==0.16.6 opencv-python==4.10.0.84
```

**mujoco** version must be 3.2.3. 3.14 fails at a joint assert in robosuite 1.4.1.

## 3. GraspGen (optional)

Needed only to rebuild the grasp candidates used by volo, core, core_detect, and mire. The candidates are already included in `outputs/grasps/`.

```bash
cd GraspGen
pip freeze | grep -v "^-e\|@ file" > /tmp/openvla_constraints.txt   # pin the current versions
pip install -c /tmp/openvla_constraints.txt torch-scatter torch-cluster -f https://data.pyg.org/whl/torch-2.2.0+cu121.html \
  torch-geometric hydra-core meshcat webdataset "tensordict<0.4" spconv-cu120 addict yapf==0.40.1 tensorboardx \
  sharedarray yourdfpy==0.0.56 pyrender pyzmq msgpack msgpack-numpy scikit-learn
pip install --no-deps -e .
cd ..
# Model weights go to checkpoints/GraspGenModels/checkpoints/ (generate_grasps.py default)
python -c "from huggingface_hub import hf_hub_download as d; [d('adithyamurali/GraspGenModels', f'checkpoints/graspgen_franka_panda{x}', local_dir='checkpoints/GraspGenModels') for x in ('.yml', '_gen.pth', '_dis.pth')]"
```

## 4. Environment variables and check

Run every command from the repository root (`VLA_MIRE/`) with the variables below set. The first part of `PYTHONPATH` finds the openvla-oft utilities (`experiments.robot.*`), the second our code (`src.*`). Our scripts run as `python src/<location>/<file>.py`.

```bash
conda activate openvla-oft
export MUJOCO_GL=egl PYTHONPATH=$PWD/openvla-oft:$PWD
export PYTHONPYCACHEPREFIX=$PWD/.cache/pycache   # no __pycache__ inside the sub-repositories
COMMON="--pretrained_checkpoint moojink/openvla-7b-oft-finetuned-libero-10 --task_suite_name libero_10"
```

The checkpoint (`moojink/openvla-7b-oft-finetuned-libero-10`) is downloaded to the Hugging Face cache on the first run. Set `HF_HOME` to change its location.

```bash
# Version check: 3.10 / 2.2.0+cu121 / 3.2.3 / 1.4.1 / 1.26.4
python -c "import sys, torch, mujoco, robosuite, numpy; print(sys.version.split()[0], torch.__version__, mujoco.__version__, robosuite.__version__, numpy.__version__)"

# Clean evaluation: 10 tasks × 50 episodes (videos of failures only)
python src/panda/oft/run_libero_eval.py $COMMON --center_crop True --num_images_in_input 2 --use_proprio True --save_videos failure
```

The clean reproduction gave 477/500 = 95.4%, close to the 94.5% LIBERO-Long (LIBERO-10) success rate reported in the OpenVLA-OFT paper ([Kim et al., RSS 2025](https://arxiv.org/abs/2502.19645)).
- **Run time**: about 2 hours on one GPU
- **Output**: `outputs/logs/libero_10/oft/EVAL-libero_10-openvla-<time>.txt`; failure videos in `outputs/videos/libero_10/oft/<run_id>/`
- **Step limit**: the default for LIBERO-10 is 520.

## 5. Preprocessing

These steps run once before the perturbed evaluation. The outputs of 5.1–5.3 (`outputs/calibration/`, `outputs/grasps/`, `outputs/clean/`) are included in the repository and need not be rebuilt. The detector bank of 2.4 is not included because of its size (about 0.9 GB).

### 5.1 Phase calibration

Without perturbations, we run 50 initial states per task and record the first target object touched, the first target object picked (= the perturbation target), the step of each, and the success step. From these records, we compute the phase lengths that set the perturbation timing.

```bash
python src/panda/oft/run_libero_calibration.py $COMMON
cp "$(ls -t outputs/calibration/libero_10/oft/CALIB-libero_10-*.json | head -1)" outputs/calibration/libero_10/oft/CALIB-libero_10.json
```

- **Outputs**
  - Calibration: `outputs/calibration/libero_10/oft/CALIB-libero_10-<time>.json` (phase lengths per task and records per episode)
  - File read by the perturbed evaluation and replay: `outputs/calibration/libero_10/oft/CALIB-libero_10.json` (set with `--calib_path`)
  - Log: `outputs/logs/libero_10/oft/CALIB-libero_10-<time>.txt`
- **Run time**: about 2 hours on one GPU
- **Scope**: all 50 initial states per task are run and recorded with their outcome. The step limit is 600, and failed episodes are also kept in the JSON.
- **Phase lengths**: the minimum over all successful episodes among the 50.
  - `full`: steps until success
  - `pre_grasp`: step at which the target first moves
  - `carry`: steps from lifting the target to releasing it

### 5.2 Grasp candidates (GraspGen)

`volo`, `core`, `core_detect`, and `mire` choose grasps from candidates prepared for each object. The candidates are included in `outputs/grasps/`; GraspGen (Section 3) is needed only to rebuild them with the command below.

```bash
python src/panda/generate_grasps.py
```

- **Input point cloud**
  - Objects: the 18 movable objects of LIBERO-10 (each asset once).
  - 2,048 points sampled **only from the outer surface** of the object mesh. The bottom is included, so grasps for tipped-over objects are also generated.
- **Generation**
  - 4 rounds of 500 grasps, 2,000 saved in total; about 3 minutes.
  - Candidates that pass the run-time filter (`conf >= 0.8`, `width <= 0.078`) range from 388 (moka pot) to 1,674 (cream cheese) per object.
- **Output**: `outputs/grasps/libero_10/<object>.npz`, sorted by GraspGen score (high to low):
  - `grasps`: N×4×4, in the object body frame
  - `conf` (score) and `width` (object width measured at the height of the finger pads)
  - `points`, `exterior_share`, `source_task`, `source_object`, `geoms`

### 5.3 Clean rollouts (mire)

MIRe imagines the arm and object trajectories without the perturbation and catches up with them to carry out the task, while handing back to the VLA early. Here we save the rollouts without perturbations; at run time, MIRe uses them as its imagined trajectories.

```bash
python src/panda/oft/collect_clean_trajectories.py $COMMON --num_workers 5
```

### 5.4 Detector bank and calibration (core_detect)

`core_detect` detects perturbations with the CoRe detector. The detector measures how far the action-head features deviate (Mahalanobis distance) from those of successful rollouts without perturbations (the detector bank). We therefore collect the bank and calibrate the trigger threshold in advance.

```bash
# Detector bank: run 50 initial states per task without perturbations and record the action-head features
python src/panda/oft/collect_detector_bank.py $COMMON --num_workers 5
# Calibration: outputs/detector/libero_10/oft/task<id>.npz, index.json, summary.json (CPU)
python src/panda/calibrate_detector.py
```
