# Project Aria Tools

Project Aria Tools is a suite of C++/Python utilities to help researchers expand
the horizons of Augmented Reality, Machine Perception and Artificial
Intelligence with [Project Aria](https://projectaria.com/). It is designed to
make it easier to use Aria data and its open datasets. It supports **both Aria
Gen1 and Aria Gen2 data**.

<div align="center">
  <a href="https://github.com/facebookresearch/projectaria_tools/releases"><img alt="Latest Release" src="https://img.shields.io/github/v/release/facebookresearch/projectaria_tools.svg" /></a>
  <a href="https://github.com/facebookresearch/projectaria_tools/blob/main/LICENSE">
  <img alt="license" src="https://img.shields.io/badge/License-Apache--2.0-blue.svg"/></a>
  <a href="https://pepy.tech/project/projectaria_tools">
  <img alt="Downloads" src="https://pepy.tech/badge/projectaria_tools"></a>
</div>

---

## 🚀 What's New in Aria Gen2

**Aria Gen2** introduces significant hardware and software improvements with
full API support in this 2.0.0 release.

### **Hardware & Sensors**

- **12MP RGB camera**, 4 CV cameras (wider FOV, HDR, front-facing stereo), 2 eye
  tracking cameras
- **New sensors**: Proximity, contact microphone, PPG health, ambient light,
  GNSS
- **6-8 hour battery life**, foldable form factor, direct interactivity with
  open-air speakers

### **On-Device Machine Perception**

On-device algorithms powered by a custom Meta co-processor:

- **Eye Tracking**, **Hand Tracking** (21 keypoints), **VIO/SLAM** (20Hz + 800Hz
  high-freq trajectory)

### **Software & Tools**

- **Unified APIs**: Same Python/C++ interface for both Gen1 and Gen2 data
- **New Tools**: `aria_rerun_viewer` (interactive 3D visualization),
  `gen2_mp_csv_exporter`, upgraded `vrs_health_check`
- **Enhanced Streaming**: USB/wireless sensor streaming with on-device
  perception signals

---

## 📖 Documentation

### **Aria Gen2 Documentation** - NEW! ✨

- **[Gen2 Documentation](https://facebookresearch.github.io/projectaria_tools/gen2/)** -
  Complete guide for Aria Gen2 data and tools
  - Research Tools APIs
  - Python/C++ examples
  - Data formats and specifications
  - On-device ML features

### **Aria Gen1 Documentation**

- **[Gen1 Documentation](https://facebookresearch.github.io/projectaria_tools/docs/intro)** -
  Legacy documentation for Aria Gen1

[![Documentation Status](https://github.com/facebookresearch/projectaria_tools/actions/workflows/publish-website.yml/badge.svg)](https://github.com/facebookresearch/projectaria_tools/actions/workflows/publish-website.yml)

---

## 📚 Interactive Python Tutorials (Google Colab)

### **Aria Gen2 Tutorials** - NEW! ✨

Comprehensive tutorials covering Aria Gen2 data processing:

1. [![Tutorial 1](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_1_vrs_data_provider_basics.ipynb)
   **VrsDataProvider Basics** - how to perform basic operations in loading and
   access data in an Aria VRS file.

2. [![Tutorial 2](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_2_device_calibration.ipynb)
   **Device Calibration** - how to work with device calibration in Aria VRS.

3. [![Tutorial 3](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_3_sequential_access_multi_sensor_data.ipynb)
   **Sequential Multi-Sensor Access** - how to use the unified queued API to
   efficiently “stream” multi-sensor data.

4. [![Tutorial 4](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_4_on_device_eyetracking_handtracking.ipynb)
   **Eye Tracking & Hand Tracking** - how to work with on-device-generated
   EyeGaze and Hand-tracking signals from Aria Gen2 glasses.

5. [![Tutorial 5](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_5_on_device_vio.ipynb)
   **On-Device VIO** - how to work with on-device-generated VIO data from Aria
   Gen2 glasses.

6. [![Tutorial 6](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_6_timestamp_alignment_in_aria_gen2.ipynb)
   **Time Synchronization** - understanding timestamp mapping in Aria data, and
   how to use timestamp mapping in multi-device recording.

7. [![Tutorial 7](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/2.1.0/examples/Gen2/python_notebooks/Tutorial_7_mps_data_provider_basics.ipynb)
   **MPS (Machine Perception Services)** - how to load and visualize output data
   from
   [Aria MP services](https://facebookresearch.github.io/projectaria_tools/gen2/ark/mps/start).

### **Aria Gen1 Tutorials**

- [![Aria VRS Data Provider](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/1.6.0/core/examples/dataprovider_quickstart_tutorial.ipynb)
  Aria VRS Data Provider

- [![Aria Machine Perception Services](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/1.6.0/core/examples/mps_quickstart_tutorial.ipynb)
  Reading and using Aria Machine Perception Services output (SLAM, Eye Tracking,
  Hand Tracking data)

---

## 🗂️ Open Datasets

### **Aria Gen2 Datasets**

- **Aria Gen2 Pilot Dataset**:
  [Dataset link](https://www.projectaria.com/datasets/gen2pilot/)
  - Multi-participant indoor and outdoor recordings
  - Full on-device outputs (eye tracking, hand tracking, VIO)
  - SubGHz-synchronized multi-device captures
  - High-quality MPS outputs (SLAM, point clouds, trajectories)

### **Aria Gen1 Datasets**

- **Aria Everyday Activities**:
  - [Dataset link](https://www.projectaria.com/datasets/aea/)
  - [![Interactive python notebook](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/main/projects/AriaEverydayActivities/examples/aea_quickstart_tutorial.ipynb)
- **Aria Digital Twin**:
  - [Dataset link](https://www.projectaria.com/datasets/adt)
  - [![Interactive python notebook](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/facebookresearch/projectaria_tools/blob/main/projects/AriaDigitalTwinDatasetTools/examples/adt_quickstart_tutorial.ipynb)
- **Aria Synthetic Environments**:
  - [Dataset link](https://www.projectaria.com/datasets/ase)

---

## How to Contribute

We welcome contributions! Go to
[CONTRIBUTING](https://github.com/facebookresearch/projectaria_tools/blob/main/.github/CONTRIBUTING.md)
and our
[CODE OF CONDUCT](https://github.com/facebookresearch/projectaria_tools/blob/main/.github/CODE_OF_CONDUCT.md)
for how to get started.

## License

Project Aria Tools are released by Meta under the
[Apache 2.0 license](https://github.com/facebookresearch/projectaria_tools/blob/main/LICENSE).
