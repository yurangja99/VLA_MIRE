# Installation

**Environment used**
- GPU: RTX 3090 24 GB × 2
- NVIDIA driver: 535.216.03
- CUDA: 12.1
- cuDNN: 8.9.2

## 1. Get the repository

Clone it from GitHub. All commands below run from the repository root `VLA_MIRE/`.

```bash
git clone --recursive https://github.com/yurangja99/vla_mire.git VLA_MIRE
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
