
# Installtion

A Guide is given [here](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html). The following contains key steps for quick installation and some additional experience.

Tested on Ubuntu 20.04 with NVIDIA Driver Version: 535.183.01 and CUDA 12.2.

## Install ISAAC SIM
Firstly, check the requirements following [this](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html#system-requirements). You may see a IOMMU warning. Disable IOMMU in your bios if there is one. If there is not, continue. Two options, Option 1: Installation using Isaac Sim pip and Option 2: Installation using Isaac Sim binaries are offered. On ubuntu 20.04, the GLIBC version is 2.31, which is lower than the minimum version requirement for option 1, so we use __Option 2__. In Option 2, we need to do Workstation Installation, including installing Cache, Nucleus, VS Code, and Isaac Sim. Every step can be done via Ominiverse Launcher.

On Linux systems, by default, Isaac Sim is installed in the directory ${HOME}/.local/share/ov/pkg/isaac_sim-*, with * corresponding to the Isaac Sim version. The version I installed is isaac-sim-4.2.0. 

Add environment varibles to bashrc. 

```
# Isaac Sim root directory
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac-sim-4.2.0"
# Isaac Sim python executable
export ISAACSIM_PYTHON_EXE="${ISAACSIM_PATH}/python.sh"
```

You can then verify the installation using 
```
${ISAACSIM_PATH}/isaac-sim.sh
```
It will take some time to open and the first time you open the isaac sim a warning about IOMMU may occur. I choose to ignore because there is no such option on my desktop.

## Install ISAAC LAB
Fork [ISAAC LAB](https://github.com/isaac-sim/IsaacLab) to your own github and clone to a local folder.
```
git clone git@github.com:isaac-sim/IsaacLab.git
```
Creating the Isaac Sim Symbolic Link
```
# enter the cloned repository
cd IsaacLab
# create a symbolic link
ln -s xxxxx_path_to_isaac_sim _isaac_sim
# For example: ln -s /home/nvidia/.local/share/ov/pkg/isaac-sim-4.2.0 _isaac_sim
```

Install Anaconda or Miniconda if you haven't installed. Then run
```
./isaaclab.sh --conda
conda activate isaaclab
sudo apt install cmake build-essential
./isaaclab.sh --install
```

Then you can test installation by
```
source ${HOME}/.local/share/ov/pkg/isaac-sim-4.2.0/setup_conda_env.sh
python source/standalone/tutorials/00_sim/create_empty.py
```
in the conda environment you created. The above command should launch the simulator and display a window with a black ground plane. You can exit the script by pressing Ctrl+C on your terminal.


## Others

### Turn Xacro to URDF
```
rosrun xacro  xacro --inorder -o dinova_description/urdf/dinova.urdf dinova_description/urdf/dinova.xacro
```

### Turn URDF to USD
```
python source/standalone/tools/convert_urdf.py  '/home/cc/chg_ws/isaac_lab/scenes/dingo_kinova/dinova.urdf' '/home/cc/chg_ws/isaac_lab/scenes/dinova_usd/dinova.usd' --make-instanceable --fix-base
```
Check convert_urdf.py for more parameters that can be set.
