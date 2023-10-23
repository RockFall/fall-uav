.. _installation-guide:

==============
Installation
==============

This document provides instructions for installing fall_uav. Please follow the steps below to get fall_uav up and running.

Prerequisites
=============

Before you install fall_uav, you'll need:

- Python 3.7 or higher
- pip (Python Package Installer)
- ROS Noetic

Optional:

- Docker (for containerized deployment)
  
Installing with pip
===================

.. important::

    This is still a work in progress. The package is not yet available on PyPI so installing with pip will not work.

You can install fall_uav directly from PyPI using pip:

.. code-block:: bash

    pip install fall_uav

From Source
===========

To install from source:

1. Clone the repository:

    .. code-block:: bash

        git clone https://github.com/your_repo/fall_uav.git

2. Navigate to the project directory:

    .. code-block:: bash

        cd fall_uav

3. Install the package:

    .. code-block:: bash

        python setup.py install

Docker Installation
===================

.. important::

    This is still a work in progress. The package is not yet available on Docker Hub so installing with Docker will not work.

If you prefer a containerized setup, a Docker image is available. You can pull and run the fall_uav Docker image as follows:

.. code-block:: bash

    docker pull your_repo/fall_uav:latest
    docker run -it your_repo/fall_uav:latest

Troubleshooting
===============

- **Issue 1**: Issues yet to be discovered

For more troubleshooting help, please refer to the `FAQ <link_to_FAQ>`_ section or `submit an issue <link_to_issue_tracker>`_.

Next Steps
==========

Once you have successfully installed fall_uav, you can proceed to the :ref:`quickstart-guide` to begin using the library.
