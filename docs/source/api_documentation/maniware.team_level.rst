.. _team_level_cooperation:

======================
Team-level Cooperation
======================

To improve the efficiency of task execution, each individual manipulator should coordinate to
allocate and perform tasks to achieve whole team cooperation. First, distributed state synchronization 
should be achieved through inter-robot communication during task execution. Second, each robot may have 
its own local observations and should support a conflict-free mechanism to resolve potential conflicts. 

Task
======================

Here we provide the documentation for the base classes that must be extended
for the various scenarios.

BaseTask
----------------------

.. autoclass:: maniware.team_level.baseTask.BaseTask
   :members:
   :undoc-members:

ManiTask
----------------------

.. autoclass:: maniware.team_level.maniTask.ManiTask
   :members:
   :undoc-members:
   :show-inheritance:

Executor
----------------------

.. autoclass:: maniware.team_level.executor.Executor
   :members:
   :undoc-members:
   :show-inheritance:

MobileManipulator
----------------------

.. autoclass:: maniware.team_level.mobileManipulator.MobileManipulator
   :members:
   :undoc-members:
   :show-inheritance:

FixedBaseManipulator
----------------------

.. autoclass:: maniware.team_level.fixedBaseManipulator.FixedBaseManipulator
   :members:
   :undoc-members:
   :show-inheritance:

Scheduler and Optimizer
=========================

Here is a detailed implementation of scheduler and optimizer

.. toctree::
   :hidden:
   :maxdepth: 1

   Scheduler <maniware.team_level.scheduler>
   Optimizer <maniware.team_level.optimizer>

