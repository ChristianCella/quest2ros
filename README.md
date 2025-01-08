# Quest2ROS  <img src="https://cdn-icons-png.flaticon.com/128/4821/4821613.png" alt="my image" width="25"/>

This repo is a fork from the original one that can be found at:
- https://github.com/Quest2ROS/quest2ros

The scripts in this repo allow to connect the Occlus quest 2 visor to a ROS framework, receive the controller positions/velocities and send haptic feedback to the controllers. All these functionalities allow to perform some teleoperation tasks on the ur5e robot, thanks to the node defined in [ros2quest.py](https://github.com/ChristianCella/quest2ros/blob/dev_chris/scripts/ros2quest.py).

---

### **Installation procedure** <a name="install"></a> 🏗️

1. Install Quest2ROS on your oculus VR headset:
 [https://quest2ros.github.io/q2r-web/](https://quest2ros.github.io/q2r-web/)

 2. Install the [ur_position_controller](https://github.com/ChristianCella/ur_position_controller/tree/master) package:

    ```
    cd (your_path_to_ws)
    git clone -b dev_chris git@github.com:ChristianCella/ur_position_controller.git
    ```

3. Clone [ROS TCP enpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) into your catkin workspace src:

    ```
    cd (your_path_to_ws)
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    ```

4. Clone [quest2ros](https://github.com/ChristianCella/quest2ros/tree/main) ROS package in your catkin workspace src folder:

    ```
    cd (your_path_to_ws)
    git clone -b dev_chris git@github.com:ChristianCella/quest2ros.git
    ```

5. Build your catkin workspace:

    `catkin build -cs`

6. Make sure ROS PC and Oculus Headset are on the same (Wi-Fi) network. To check the IP address of your Wi-Fi network go in Settings -> IPv4 address.

7. Set <YOUR_IP> and the same port in the VR (after selecting the application Quest2Ros) and press **apply** (for example, in my case it was 192.168.1.22).

8. Start ROS TCP endpoint (replace <YOUR_IP>)

    ```
    roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=<YOUR_IP> tcp_port:=10000
    ```

9. You should see [INFO] in the terminal that the connection is established

10. Run [ros2quest.py](https://github.com/ChristianCella/quest2ros/blob/dev_chris/scripts/ros2quest.py), which allows to write the pose on the toopic '/desired_pose':

    ```
    rosrun quest2ros ros2quest.py
    ```

---

### **Teleoperation** <a name="teleop"></a> 🎮
To use the package for teleoperation, open 4 terminals:

1. **Terminal for the build:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    cd (your_path_to_ws)
    catkin build -cs
    ```
    The command '-cs' skips all the packages that cannot be built.


2. **Terminal to bring-up the robot:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    roslaunch ur5e_mics_bringup raw_control.launch
    ```
    If you want to connect to the real robot (after following the instructions to ping the robot and switching to remote control provided at this [link](https://github.com/MerlinLaboratory/ur5e_mics)), simply execute:
     ```
    roslaunch ur5e_mics_bringup raw_control.launch gazebo_sim:=false gripper_name:='hande'
    ```

3. **Terminal to bringup the communication visor-ROS:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    roslaunch quest2ros quest2ros.launch ip:=<YOUR_IP>
    ```

4. **Terminal for the position controller:**
    ```
    source (your_path_to_ws)/devel/setup.bash
    rosrun ur_position_controller position_controller
    ```

IMPORTANT: for how the frame ```teleop_link``` has been defined, the orientation of the visor is very important: since you are not wearing it during the operations, if you want the robot to mirror your movements you have to be facing the visor while moving the remote in the space. On the contrary, the movements of the remote will be mapped in the wrong way.

---

### **Coordinate frame alignment** <a name="alignment"></a> 🖼️

By pressing A + B on the right and X+Y on the left, the relative coordinate frame gets aligned to the current controller position.
To use this for robot teleop allign the controller with the base frame of the robot and press the buttons.

---

### **Troubleshooting** <a name="troubleshooting"></a> 🔫
In case the TCP-endpoint is not connecting to the VR app, try to allow the connection through the port you have specified (10000 in this example) with the following command:

```
sudo iptables -I INPUT -p tcp -m tcp --dport 10000 -j ACCEPT
```

---

### **BibiTex** <a name="bibitex"></a> 📔

```
@inproceedings{@software{q2r2023,
  title={Quest2ROS: An App to Facilitate Teleoperating Robots},
  author={Welle, Michael C and Ingelhag, Nils and Lippi, Martina and Wozniak, Maciej K. and Gasparri, Andrea and Kragic, Danica},
  url = {https://quest2ros.github.io/q2r-web/},
  version = {1.0},
  date={2023-12-01}
}
```




