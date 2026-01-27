# SOFA Tibial Cutting Guide Simulation

Physics simulation for surgical cutting guide placement on the distal anterior tibia using the SOFA framework. Models guide seating, pin constraints, and saw-slot mechanics to verify design robustness under realistic intraoperative conditions.

## Purpose

- Validate guide seating stability under perturbation
- Quantify sensitivity to pin insertion angle errors
- Verify saw slot constrains blade under vibration
- Cross-validate against 6dof-mc ICP results

## Installation

### Prerequisites

- Python 3.10+
- Conda (recommended)

### Environment Setup

```bash
conda env create -f environment.yml
conda activate sofa-tibia-guide
```

### SOFA Installation

SOFA must be installed from pre-built binaries (not available via conda/pip).

1. **Install system dependencies:**

```bash
sudo apt install libopengl0
```

2. **Install Python dependencies for SofaPython3:**

```bash
# For Python 3.10 (use SOFA v24.06)
pip install numpy scipy pybind11==2.11.1

# For Python 3.12 (use SOFA v24.12 or v25.06)
pip install numpy scipy pybind11==2.12.0
```

3. **Download SOFA binary release:**

Download from [SOFA Releases](https://github.com/sofa-framework/sofa/releases):
- **Python 3.10**: Use SOFA v24.06.00
- **Python 3.12**: Use SOFA v24.12.00 or v25.06.00

```bash
# Example for v24.06.00 (Python 3.10)
wget https://github.com/sofa-framework/sofa/releases/download/v24.06.00/SOFA_v24.06.00_Linux.zip
unzip SOFA_v24.06.00_Linux.zip -d ~/sofa
export SOFA_ROOT=~/sofa/SOFA_v24.06.00_Linux
```

4. **Set environment variables:**

```bash
export PYTHONPATH=$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages:$PYTHONPATH
export LD_LIBRARY_PATH=$SOFA_ROOT/lib:$LD_LIBRARY_PATH
```

Add these to your `.bashrc` or conda environment activation script.

5. **Verify installation:**

```bash
python -c "import Sofa; print('SOFA imported successfully')"
```

**Version compatibility**: SOFA v24.06+ (match Python version to SOFA release)

## Project Structure

```
sofa-tibia-guide/
├── docs/                   # Documentation
│   ├── user-needs.md       # Stakeholder needs and use cases
│   ├── requirements.md     # Functional/non-functional requirements
│   ├── architecture.md     # System design and SOFA scene graph
│   └── asset-pipeline.md   # Mesh preparation workflow
├── scenes/                 # SOFA scene definitions
│   └── mvm_seating.py      # Tier 1: rigid contact + friction
├── experiments/            # Batch experiment runners
│   └── run_disturbance.py  # Headless experiment runner
├── src/tibia_guide/        # Core library
│   ├── assets/             # Asset loading and transforms
│   ├── metrics/            # Pose and contact metrics
│   └── config.py           # Simulation configuration
├── tests/                  # Test suite
│   ├── unit/               # Unit tests (no SOFA required)
│   └── integration/        # Integration tests (SOFA required)
├── assets/                 # Mesh files (not in repo)
└── outputs/                # Generated outputs (gitignored)
```

## Usage

### Running Tests

Unit tests (no SOFA required):

```bash
pytest tests/unit/
```

Full test suite (SOFA required):

```bash
pytest
```

### Running Simulations

```bash
python experiments/run_disturbance.py --config config.yaml
```

### Quick Start

```python
from tibia_guide.assets.loader import load_mesh
from tibia_guide.assets.transform import apply_transform
from scenes.mvm_seating import create_scene

# Load meshes
tibia = load_mesh("assets/tibia.stl")
guide = load_mesh("assets/guide.stl")

# Create and run simulation
root = create_scene(tibia_mesh=tibia, guide_mesh=guide)
```

## Documentation

- [User Needs](docs/user-needs.md) - Stakeholder requirements
- [Requirements](docs/requirements.md) - Functional specifications
- [Architecture](docs/architecture.md) - System design
- [Asset Pipeline](docs/asset-pipeline.md) - Mesh preparation

## License

Proprietary - Internal use only
