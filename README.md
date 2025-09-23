# 🔹 Εγκατάσταση Gazebo11 + ROS2 + ArduPilot + MAVROS

## 1. Προαπαιτούμενα

Σε Ubuntu 20.04 (για ROS2 Foxy) ή Ubuntu 22.04 (για ROS2 Humble):

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install git wget curl build-essential cmake python3-colcon-common-extensions -y
```

---

## 2. Εγκατάσταση ROS 2

### Ubuntu 22.04 → ROS 2 Humble

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Εγκατάσταση Gazebo11 (Classic)

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install gazebo11 libgazebo11-dev -y
```

Δοκιμή:

```bash
gazebo
```

---

## 4. Εγκατάσταση ArduPilot

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

Build για SITL:

```bash
./waf configure --board sitl
./waf copter
```

---

## 5. Εγκατάσταση ardupilot\_gazebo plugin

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

Ρύθμιση environment variables:

```bash
echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=~/ardupilot_gazebo/build:${LD_LIBRARY_PATH}' >> ~/.bashrc
source ~/.bashrc
```

---

## 6. Εγκατάσταση MAVROS

```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras -y
```

Εγκατάσταση geographiclib datasets:

```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

---

# ✅ Flow Εκκίνησης

1. Άνοιξε Gazebo με Iris world:

   ```bash
   cd ~/ardupilot_gazebo/worlds
   gazebo --verbose iris_arducopter_runway.world
   ```

2. Τρέξε SITL:

   ```bash
   cd ~/ardupilot
   ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
   ```

3. Άνοιξε MAVROS:

   ```bash
   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14550@localhost:14550
   ```

4. (Αν έχεις camera plugin) κάνε bridge:

   ```bash
   ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
   ```

---
Στο SITL terminal:

```bash
mode GUIDED
ARM THROTTLE
TAKEOFF 10
```

Τρεξε σε ξεχωριστό terminal το αρχείο my_package/my_package/twist_publisher1 για το φέρεις το drone σε θέση που βλέπει το αντικείμενο.Τερματισέ το και έπειτα μπορείς να τρέξεις:
   - Ένα από τα αρχεία στον φάκελο Simulations που αφορούν την αυτόνομη παρακολούθηση και πλοήγηση του αντικειμένουν μέσω της κάμερας του drone 
   - my_package/my_package/object_trajectory για να μετακινήσεις το αντικείμενο
   - my_package/my_package/object_tracking για να παρακολουθείς το σφάλμα θέσης και ταχύτητας drone/αντικειμένου
   - Rviz2 για τα image topic που δημιουργούνται από το models/iris_with_ardupilot/model.sdf
