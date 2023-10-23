.. _robotics-api-guide:

==============
Robotics API
==============

The ``robotics_api`` module in the fall_uav library acts as an abstraction layer between the high-level UAV control logic and the low-level middleware, like ROS (Robot Operating System), MRS (Multi-Robot Systems), and others. This guide will outline the structure and functionality of this crucial component.

Introduction
============

The ``robotics_api`` module serves as a middleware abstraction layer, providing a unified API to interact with various underlying systems like ROS, MRS, and others. At the heart of this module is the `RoboticsAPI` class, which manages and directs calls to the appropriate middleware.

RoboticsAPI Class
=================

The ``RoboticsAPI`` class acts as the main interface for middleware interactions and dynamically routes calls to the appropriate subsystem based on the `system_type` provided during initialization.

.. code-block:: python

    from fall_uav.robotics_api import RoboticsAPI

    # Initialize with ROS as the system type
    robotics_api = RoboticsAPI(system_type='ros')

    # Example methods
    robotics_api.arm()
    robotics_api.takeoff()
    robotics_api.go_to(x, y, z, w)

The `RoboticsAPI` class currently supports the following methods:

- ``arm()``: Arms the drone.
- ``disarm()``: Disarms the drone.
- ``set_flight_mode(mode)``: Sets the flight mode.
- ``takeoff()``: Initiates takeoff.
- ``land()``: Initiates landing.
- ``go_to(x, y, z, w)``: Moves the drone to a specific location.
- ``publish_velocity(x, y, z, w)``: Publishes velocity commands.
- ``update_pose()``: Updates the drone's current pose.

Each of these methods routes the call to the appropriate API class (`ROS_API`, `MRS_API`, etc.) based on the `system_type`.

Middleware Classes
==================

ROS_API Class
-------------

The ``ROS_API`` class provides an interface for ROS-specific functionality.

.. code-block:: python

    from fall_uav.robotics_api import ROS_API

    # Initialize
    ros_api = ROS_API()

    # Use ROS_API methods
    ros_api.some_ros_method()

MRS_API Class
-------------

The ``MRS_API`` class is tailored for Multi-Robot Systems.

.. code-block:: python

    from fall_uav.robotics_api import MRS_API

    # Initialize
    mrs_api = MRS_API()

    # Use MRS_API methods
    mrs_api.some_mrs_method()

And so on for other middleware like `DDS_API`, `MQTT_API`, etc.


Next Steps
==========

For practical examples that integrate the Robotics API, see the :ref:`quickstart-guide`. For more in-depth information about the controllers, refer to the :ref:`controllers-guide`.
