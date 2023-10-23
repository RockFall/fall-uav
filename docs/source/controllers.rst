.. _controllers-guide:

==============
Controllers
==============

The ``controllers`` module in the fall_uav library is designed to manage various aspects of drone control, from basic movement to complex tasks like navigation and stabilization. This guide will walk you through the available controllers and how to use them.

.. important::

    This page will change completely on the following weeks. Please check back later.


DroneController
===============

The ``DroneController`` class serves as the main interface for controlling a UAV.

.. code-block:: python

    from fall_uav.controllers import DroneController

    # Initialize the drone controller
    controller = DroneController()

MotionController
================

The ``MotionController`` class manages the drone's movements.

.. code-block:: python

    from fall_uav.controllers import MotionController

    # Initialize
    motion_controller = MotionController(controller)

    # Command the drone to move
    motion_controller.move_to(50, 50, 20)

NavigationController
====================

The ``NavigationController`` class is used for higher-level navigation tasks.

.. code-block:: python

    from fall_uav.controllers import NavigationController

    # Initialize
    nav_controller = NavigationController(controller)

    # Navigate to a waypoint
    nav_controller.navigate_to_waypoint(latitude, longitude, altitude)

StabilizationController
=======================

The ``StabilizationController`` class manages the drone's stability.

.. code-block:: python

    from fall_uav.controllers import StabilizationController

    # Initialize
    stab_controller = StabilizationController(controller)

    # Stabilize the drone
    stab_controller.stabilize()

And so on for other controllers like `VisionController`, `WaypointManager`, etc.

Next Steps
==========

For a practical example that uses these controllers, see the :ref:`quickstart-guide`. For more details on the API, refer to the :ref:`api-reference`.
