![GitHub issues open](https://img.shields.io/github/issues/kuralme/mpc_ros2)
![GitHub forks](https://img.shields.io/github/forks/kuralme/mpc_ros2)
![GitHub stars](https://img.shields.io/github/stars/kuralme/mpc_ros2)
![GitHub license](https://img.shields.io/github/license/kuralme/mpc_ros2)


# Nonlinear Model Predictive Controller ROS2


This repository aims to implement a path tracking algorithm for mobile robots using Nonlinear Model Predictive Control (NMPC), a type of optimal controller built on ROS2. NMPC is known for its powerful and efficient performance compared to other optimal controllers, thanks to its predictive capabilities and ease of constraint handling.

The project integrates the [IPOPT solver](https://coin-or.github.io/Ipopt/), which is ideal for solving large-scale nonlinear optimization problems. IPOPT is robust, scalable, and fast, making it well-suited for real-time applications like trajectory optimization. Its open-source nature ensures transparency and customization, allowing for efficient and reliable solutions in dynamic environments.
<!-- TODO this repository supports both the local planner plugin and standalone node configurations. Therefore it supports the local planner plugin and standalone node as well. -->

<!-- [![Video Label](http://img.youtube.com/vid)] -->



### Dependencies
1. ROS2 Humble depends:
    ```
    rosdep install --from-paths <your/mpc_ros2/dir> -y --ignore-src
    ```

2. CppAD
    ```
    sudo apt install cppad gfortran  
    ```

3. Ipopt: refer to the official tutorial ["ipopt_install"](https://coin-or.github.io/Ipopt/INSTALL.html).

4. Gazebo Sim ([Harmonic](https://gazebosim.org/docs/harmonic/getstarted/))

## Launch

### Start MPC in simulation: 

```
ros2 launch mpc_ros2 mpc_planner.launch.py
```

## TODO
- [ ] local planner plugin


