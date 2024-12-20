# Nonlinear Model Predictive Controller ROS2


This repository aims to implement a path tracking algorithm for mobile robots using Nonlinear Model Predictive Control (NMPC), a type of optimal controller built on ROS2. NMPC is known for its powerful and efficient performance compared to other optimal controllers, thanks to its predictive capabilities and ease of constraint handling.

The project integrates the [IPOPT solver](https://coin-or.github.io/Ipopt/), which is ideal for solving large-scale nonlinear optimization problems. IPOPT is robust, scalable, and fast, making it well-suited for real-time applications like trajectory optimization. Its open-source nature ensures transparency and customization, allowing for efficient and reliable solutions in dynamic environments.
<!-- TODO this repository supports both the local planner plugin and standalone node configurations. Therefore it supports the local planner plugin and standalone node as well. -->

<!-- [![Video Label](http://img.youtube.com/vid)] -->



### Dependencies
1. ROS2 Jazzy depends:
    ```
    rosdep install --from-paths <your/mpc_ros2/dir> -y --ignore-src
    ```

2. CppAD: refer to the [tutorial](https://cppad.readthedocs.io/latest/Install.html)

3. Ipopt: refer to the official ["tutorial"](https://coin-or.github.io/Ipopt/INSTALL.html).


## Launch

### Start MPC controller node:

```
ros2 launch mpc_ros2 mpc_planner.launch.py
```

## TODO
- [ ] tests on sim
- [ ] local planner plugin