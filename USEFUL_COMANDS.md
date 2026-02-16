# ORB_SLAM3 Execution & Configuration Guide

---

## 🚀 Execution Commands

### Run with EuRoC Format (Monocular)

```bash
cd /home/samuelecentanni/github/ORB_SLAM3 && \
LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH \
./Examples/Monocular/mono_euroc \
./Vocabulary/ORBvoc.txt \
./Examples/Monocular/EuRoC.yaml \
~/Datasets/EuRoC/vicon_room1/V1_01_easy \
~/Datasets/EuRoC/vicon_room1/V1_01_easy/mav0/cam0/timestamps.txt

```

### Run with Custom Format (Monocular-Inertial)

```bash
cd /home/samuelecentanni/github/ORB_SLAM3 && \
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH && \
./Examples/Monocular-Inertial/mono_inertial_euroc \
    ./Vocabulary/ORBvoc.txt \
    ./Examples/Monocular-Inertial/Insta360.yaml \
    /home/samuelecentanni/Desktop/BuchsIT/floor_1/2025-05-05/run_1/dataset \
    /home/samuelecentanni/Desktop/BuchsIT/floor_1/2025-05-05/run_1/dataset/timestamps.txt

```

---

## 🛠 Required Modifications & Fixes

### Build Configuration

* **C++ Standard:** Updated `CMakeLists.txt` to prefer **C++14** to resolve compatibility issues with the `sigslot` library in Pangolin.
* **Dependency Resolution:** Built the core library `libORB_SLAM3.so` and linked all necessary dependencies.
* **Clock Update:** Replaced deprecated `std::chrono::monotonic_clock` with `std::chrono::high_resolution_clock` to support C++17+ environments.

### Viewer Activation

The GUI Viewer was enabled by modifying line 83 in `mono_euroc.cc`:

* **Original:** `ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, false);`
* **Updated:** `ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);`

---

## 📊 Data Preparation

### Extraction from ROS2 Bag

```bash
python3 extract_rosbag2.py \
  --bag /home/samuelecentanni/Desktop/BuchsIT/floor_1/2025-05-05/run_1/rosbag/rosbag_0.db3 \
  --output /home/samuelecentanni/Desktop/BuchsIT/floor_1/2025-05-05/run_1/dataset \
  --image_topic /cam0/image_raw/compressed \
  --imu_topic /imu/data_raw

```

### Timestamp Normalization (ns to sec)

Converts the first column of the trajectory file from nanoseconds to decimal seconds:

```bash
awk '{printf "%.6f", $1/1e9; for(i=2;i<=NF;i++) printf " %s", $i; print ""}' \
/home/samuelecentanni/media/orbslam_predictions/floor_2_2025-12-03_run_1.txt > /tmp/traj_sec.txt && \
mv /tmp/traj_sec.txt /home/samuelecentanni/media/orbslam_predictions/floor_2_2025-12-03_run_1.txt && \
head -3 /home/samuelecentanni/media/orbslam_predictions/floor_2_2025-12-03_run_1.txt

```