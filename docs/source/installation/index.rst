.. _installation:

===================
Installation
===================

In this page, we provide the installation procedure for the **ManiWare** package.
Before the installation, please ensure your system meet the following requirements:

- Ubuntu 20.04+
- Pybullet 3.2.1
- Python 3.7.5

ManiWare download 
--------------------------------------------
The **ManiWare** supports Ubuntun 20.04 system with Python3 and Pybullet simulation engine.
Please refer to the `Pybullet website <https://pybullet.org/wordpress/>`_ for a comprehensive
tutorial on how to install Pybullet. 

The **Maniware** use the Pybullet as the simulation engine, which can customize different types
of robotic model according to requirements. Install Pybullet is simple

.. code-block:: bash

    pip3 install pybullet --upgrade --user

To download **ManiWare**, clone the package from github repository:

.. code-block:: bash

	git clone https://github.com/sundycoder/ManiWare.git .
	

Installation of required Python packages
--------------------------------------------
**ManiWare** requires a set of Python packages that can be installed by running
(inside the root directory of your workspace):

.. code-block:: bash

	pip3 install -r requirements.txt
