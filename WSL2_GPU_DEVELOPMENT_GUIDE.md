# ROS 2 on Windows — WSL2 + Ubuntu 22.04 + GPU Setup Guide

A step-by-step guide for setting up a ROS 2 development environment on Windows using WSL2, VS Code, and hardware GPU acceleration.

---

## Step 1 — Install WSL2 and Ubuntu 22.04

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-22.04
```

This installs WSL2 and Ubuntu 22.04 in one command. When it finishes, **restart your PC**.

On first launch Ubuntu will ask you to create a Linux username and password — this is your Linux account, independent from Windows.

Verify WSL2 is active:

```powershell
wsl --list --verbose
```

You should see `VERSION 2` next to Ubuntu-22.04. If it shows `1`, convert it:

```powershell
wsl --set-version Ubuntu-22.04 2
```

Once inside Ubuntu, update the system before doing anything else:

```bash
sudo apt update && sudo apt upgrade -y
```

---

## Step 2 — Connect VS Code to WSL

Install the **WSL** extension in VS Code (publisher: Microsoft).

Then open your Ubuntu terminal, go to your project folder, and launch VS Code from there:

```bash
cd ~/your_ros2_workspace
code .
```

VS Code opens on Windows but everything — the terminal, extensions, and compiler — runs inside Ubuntu. The bottom-left status bar will show **WSL: Ubuntu-22.04** in green when connected.

> Keep your code inside the Linux filesystem (`~/your_project/`) and **not** under `/mnt/c/`. Accessing the Windows drive from WSL adds I/O overhead that slows down `colcon build` significantly.

---

## Step 3 — Activate the GPU (The Step Most People Miss)

This is the most important step and the one that is most often skipped.

By default, WSL2 does **not** use your dedicated NVIDIA GPU. Without this step, Gazebo and RViz2 fall back to software rendering (LLVMpipe), which produces very low frame rates (5–12 FPS) and makes real-time simulation unusable.

On a laptop with both an Intel integrated GPU and an NVIDIA discrete GPU, the system will silently pick the Intel GPU unless you configure it otherwise. You must do this manually.

### 3.1 — Enable Hardware-Accelerated GPU Scheduling in Windows

Go to:

```
Settings → System → Display → Graphics → Change default graphics settings
```

Turn on **Hardware-accelerated GPU scheduling**, then **reboot Windows**.

This allows the GPU to manage its own command queue directly instead of routing through the Windows CPU scheduler, which reduces rendering latency for WSL2 applications.

### 3.2 — Update your NVIDIA driver

WSL2 GPU support requires driver version **528.24 or later**. Update via GeForce Experience or download directly from the NVIDIA website.

You do **not** install any driver inside Ubuntu — the Windows driver is automatically shared with WSL2.

### 3.3 — Verify the GPU is visible inside Ubuntu

Open your Ubuntu terminal and run:

```bash
nvidia-smi
```

A table showing your GPU name, driver version, and CUDA version means the GPU is correctly linked to WSL2. If the command is not found, your Windows driver is outdated.

### 3.4 — Force the GPU for ROS 2 and Gazebo

Add these lines to your `~/.bashrc` so they apply automatically to every new terminal:

```bash
# Force the dedicated NVIDIA GPU (prevents falling back to Intel iGPU)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Disable V-Sync (removes the FPS cap tied to the display refresh rate)
export __GL_SYNC_TO_VBLANK=0
export vblank_mode=0

# Tell Gazebo to use the Ogre2 hardware rendering backend
export IGN_RENDERING_ENGINE=ogre2
```

Apply immediately without restarting the terminal:

```bash
source ~/.bashrc
```

### 3.5 — Confirm hardware rendering is active

```bash
sudo apt install -y mesa-utils
glxinfo | grep renderer
```

The output should show your NVIDIA GPU name. If it shows `llvmpipe` or `softpipe`, the GPU is still not being used — double-check that Hardware-Accelerated GPU Scheduling is enabled and that you rebooted after enabling it.

---

## Result

| | Without GPU activation | With GPU activation |
|---|---|---|
| Gazebo FPS | 5–12 FPS (software) | 30–60+ FPS (hardware) |
| RViz2 | Laggy / crashes | Smooth real-time |
| CUDA for AI training | Not available | Available |
| Renderer | LLVMpipe (CPU) | NVIDIA OpenGL |
