# FallUAV

A comprehensive Python-based UAV management system designed for controlling and automating drone missions via MAVROS in a ROS environment. Initially conceptualized for AVANT (UFMG's UAVs team) for the Brazilian Robotics Competition (CBR), this library has evolved to manage various UAV operations beyond just waypoint navigation.

## Overview

FallUAV provides a robust framework for UAV mission planning, real-time drone status monitoring, and intuitive drone operation commands. It ensures precision, efficiency, and versatility in managing UAVs for complex missions.

## Features

- **Easy Initialization**: Begin your drone operations with a few simple configurations.
- **Mission Management**: Define and execute intricate UAV missions with ease.
- **Real-time Monitoring**: Stay updated with your drone's position, status, and movement.
- **Enhanced Control**: Issue commands like takeoff, landing, move-to, and even specific operations like QR scanning.
- **Versatile Integration**: Built to incorporate smoothly into MAVROS and ROS-based UAV setups.
- **PID-Controlled Movement**: Precision movement with PID control to handle complex flight patterns and maneuvers.
- **Command-Line Interface**: Easily interact with your UAV and send commands through a user-friendly CLI.
- **Flexibility**: Designed for scalability, making it simple to add new features and functionalities as required.

## Installation

```bash
git clone https://github.com/RockFall/fall-uav.git
cd fall-uav
```

## Usage
The main entry point is through the CLI avantCMD.py, providing an interactive environment to manage your UAV:
```bash
python avantCMD.py
```

## Mission JSON Files
The mission.json files play a pivotal role in the FallUAV system, providing a structured way to define complex UAV missions. These files allow for sequential and granular control of UAV actions, making it simple to automate a wide array of operations.

### Structure
Each mission is a sequence of steps, and each step is defined as a JSON object within an array. The object contains an action field denoting the type of step (e.g., "takeoff", "move_to", "land", etc.) and a params field detailing the specific parameters for that action.

Example:
```json
[
    {
        "action": "takeoff",
        "params": {
            "delay": 5.0
        }
    },
    {
        "action": "move_to",
        "params": {
            "waypoints": [[0.0, 0.0, 2.0, 1.57]]
        }
    },
    {
        "action": "land"
    }
]
```

In the above, the UAV will take off, delay for 5 seconds, move to a specific waypoint, and then land.

### Creating Your Own
To define your mission:

1. Start with an empty JSON array [ ].
2. Add JSON objects for each step of your mission.
3. Specify the action and any required params for that action.
4. Save the file with a .json extension and provide it as input to the FallUAV system.

Refer to the provided documentation for a comprehensive list of possible actions and their respective parameters.

## Documentation
Detailed documentation can will be available in the future.

## Contributing
We welcome contributions! Please see our contributing guidelines for more details (yet to be written).

## License
This project is licensed under the MIT License.

## Acknowledgments
Thanks to MAVROS and ROS communities for their extensive tools and resources.
Special thanks to thoseV/ of AVANT who contributed to the development and enhancement of this library.
