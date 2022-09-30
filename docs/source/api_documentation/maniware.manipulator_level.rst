.. _mani_level_motion_control:

=================================
Manipulator-level Motion Control
=================================

In a manipulator team, each manipulator is constructed with end-effectors and rigid links 
connected by joints, and a mobile manipulator is constructed by deploying a manipulator on 
a chassis. To enable robust motion control, the middleware should provide manageable and 
flexible motion controllers for different components of manipulators. Besides, the primitives 
of motion controllers should be easy to reuse and extend with the updated requirements of systems.

Base classes
======================

Here we provide the documentation for the base classes that must be extended
for the various scenarios.

BaseComponent
----------------------

.. autoclass:: maniware.mani_level.component.baseComponent.BaseComponent
   :members:
   :undoc-members:

BaseController
----------------------

.. autoclass:: maniware.mani_level.controller.baseController.BaseController
   :members:
   :undoc-members:
   :show-inheritance:

Components and Controllers
===========================

Here is a detailed implementation of components and controllers

.. toctree::
   :maxdepth: 1

   Component <maniware.mani_level.component>
   Controller <maniware.mani_level.controller>

