### Installation
Follow the linked instructions to install and build the PX4 simulator: [PX4 Installation] (https://docs.px4.io/main/en/dev_setup/building_px4)

Follow the instructions to download QGroundControl: [QGroundControl Installation] (https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

### Set Up
* Download drone_control.py
* In terminal go to directory with PX4-Autopilot folder then run the following commands.
```bash
cd PX4-Autopilot
PX4_SYS_AUTOSTART=10040 make px4_sitl_default none
```
* Open QGroundControl
* In a new terminal window run the following command
```bash
python drone_control.py
```
* A window should open with the keyboard controls to run the simulation.
  
