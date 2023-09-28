# FallWaypoints

A Python-based interface library for controlling drones using waypoints via MAVROS in a ROS environment. This project is being developed with the purpose of being incorporated into the system used by AVANT (UFMG's UAVs team) on the Brazilian Robotics Competition (CBR) but may have continuous development for future projects.

## Overview

`FallWaypoints` offers streamlined waypoint management, real-time drone status monitoring, and intuitive motion control commands, ensuring precise and efficient navigation for UAVs.

## Features

- **Easy Initialization**: Begin your drone operations with a few simple configurations.
- **Waypoint Management**: Set, modify, and traverse a series of waypoints with ease.
- **Real-time Monitoring**: Always be aware of your drone's position and status.
- **Enhanced Control**: Commands to manage your drone's operations seamlessly.
- **ROS & MAVROS Compatibility**: Built to integrate smoothly with your existing ROS infrastructure.

## Installation

```bash
git clone https://github.com/your-github-username/fall-waypoints.git
cd fall-waypoints
```

## Usage
```python
from fall_waypoints import FallWaypoints


# Initialize
manager = FallWaypoints(namespace='my_namespace', frequency=10)

# Set waypoints
waypoints = [(x1, y1, z1, yaw1), (x2, y2, z2, yaw2), ...]
manager.set_waypoints(waypoints)

# Start movement
manager.start_movement()
```

## Documentation
Detailed documentation can will be available in the future.

## Contributing
We welcome contributions! Please see our contributing guidelines for more details (yet to be written).

## License
This project is licensed under the MIT License.

## Acknowledgments
Thanks to MAVROS and ROS communities for their extensive tools and resources.
Special thanks to thoseV/ of AVANT who contributed to the development and enhancement of this library.
