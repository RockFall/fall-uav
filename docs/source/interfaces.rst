.. _interfaces-guide:

==============
Interfaces
==============

The ``interfaces`` module in the fall_uav library provides various user interfaces for interacting with the UAV system. Currently, a Command Line Interface (CLI) is available, with plans for a frontend web interface in the future.

Command Line Interface (CLI)
============================

The CLI is implemented in the ``cli.py`` file and offers a simple way to control the drone and execute missions. Below are the available commands:

- ``start``: Starts the mission (Takeoff -> Waypoints -> Landing).
- ``reset``: Resets the mission to its starting point.
- ``takeoff``: Initiates drone takeoff.
- ``land``: Initiates drone landing.
- ``exit``: Exits the program.
- ``help``: Shows the list of available commands.

.. code-block:: bash

    --- Iniciando drone... ----
    c> start
    Mission completed

.. note:: The CLI is still under development, and contributions for its improvement are welcome.

Future: Web Interface
=====================

A web-based frontend interface is planned for future releases. This interface will offer more robust controls and data visualization features, providing a more user-friendly way to interact with the UAV system.

Next Steps
==========

For a practical example using the CLI interface, see the :ref:`quickstart-guide`. For more details on the controllers that work behind the scenes, refer to the :ref:`controllers-guide`.
