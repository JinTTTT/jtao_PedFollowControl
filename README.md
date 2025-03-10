# Practical Pedestrian Interaction and Tracking Control System

This system is developed based on **ROS2 Humble** and **Ubuntu 22.04**, designed for pedestrian interaction and tracking control in autonomous driving scenarios. It must be used in conjunction with a functioning perception system.

## System Inputs and Outputs

### Inputs:
- **Relative pedestrian position** (provided by the perception system).
- **Vehicle status data** (steering angle, velocity, etc.) from the CAN bus.
- **Controller status** (optional, required if using a gamepad or similar controller).

### Outputs:
- **Target velocity**
- **Target steering angle**

## Project Structure

The core code is organized in the `src` directory with the following key components:

- `fake_bus.py` & `fake_pedestrian.py`:  
  Simulate vehicle and pedestrian data for testing purposes when real sensor data is unavailable.

- `get_gesture.py`:  
  Filters and selects the most reliable detected gesture input for further use.

- `get_nearest_obj.py`:  
  Filters pedestrian detections and selects the nearest pedestrian as the tracking target, particularly useful in multi-target scenarios.

- `gui_node.py`:  
  Provides a visualization GUI for monitoring system status, vehicle and pedestrian positions, as well as real-time commands.
  ![Screenshot 2025-03-10 at 21 35 34](https://github.com/user-attachments/assets/a75ce15f-a455-47fa-a8a4-66649719639a)

- `motion_estimator.py`:  
  Estimates the global coordinates of pedestrians and computes vehicle positions and orientations based on provided inputs.

- `ped_follow.py`:  
  Core logic of the system; responds to gesture inputs and implements pedestrian tracking and vehicle control functionalities.
