# Quest2ROS  <img src="https://cdn-icons-png.flaticon.com/128/4821/4821613.png" alt="my image" width="25"/>

This repo is a fork from the original one that can be found at:
- https://github.com/Quest2ROS/quest2ros

The scripts in this repo allow to connect the Occlus quest to ROS and receive the controller Position/velocities and send haptic feedback to the controllers. To use the original repo linked above you can checkout to the main branch of this one instead.

---

### **Installation procedure** <a name="install"></a> 🏗️

1. Install Quest2ROS on your oculus VR headset:
 [https://quest2ros.github.io/q2r-web/](https://quest2ros.github.io/q2r-web/)

2. Clone [ROS TCP enpoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint) into your catkin workspace src:

    ```
    cd (your_path_to_ws)
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    ```

3. Clone quest2ros ROS package in your catkin workspace src:

    ```
    https://github.com/ChristianCella/quest2ros.git
    ```

4. Build your catkin workspace:

    `catkin build -cs`

5. Make sure ROS PC and Oculus Headset are on the same (Wi-Fi) network. To check the IP address of your Wi-Fi network go in Settings -> IPv4 address.

6. Set <YOUR_IP> and the same port in the VR (after selecting the application Quest2Ros) and press **apply** (for example, in my case it was 192.168.1.22).

7. Start ROS TCP endpoint (replace <YOUR_IP>)

    ```
    roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=<YOUR_IP> tcp_port:=10000
    ```

8. You should see [INFO] in the terminal that the connection is established

9. Run [ros2quest.py](https://github.com/ChristianCella/quest2ros/blob/dev_chris/scripts/ros2quest.py), which allows to write the pose on the toopic '/desired_pose':

    ```
    rosrun quest2ros ros2quest.py
    ```
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




