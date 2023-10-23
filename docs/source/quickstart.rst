.. _quickstart-guide:

==============
Quickstart Guide
==============

This Quickstart Guide will show you how to set up and run a basic example using the fall_uav library. This guide assumes that you've already installed the library. If you haven't, please refer to the :ref:`installation-guide`.

Initialization
==============

First, let's import the necessary modules from the fall_uav library.

.. code-block:: python

    from fall_uav import DroneController, Mission

Basic Drone Control
===================

Here's how you can initialize the drone controller and set a simple mission.

.. code-block:: python

    # Initialize the drone controller
    controller = DroneController()

    # Create a mission
    mission = Mission(controller)
    
    # Add waypoints or tasks
    mission.add_waypoint(latitude, longitude, altitude)
    mission.add_task('take_photo')

    # Execute the mission
    mission.execute()

Run the Example
===============

Save the above code in a file named `simple_mission.py` and run it.

.. code-block:: bash

    python simple_mission.py

You should see the drone execute the mission and perform the tasks you've defined.

Next Steps
==========

Congratulations, you've successfully run your first mission with the fall_uav library!

