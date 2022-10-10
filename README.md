<div align="center">
  <img src="docs/source/_static/logo.png" width="700"/>
</div>

------------
# ManiWare [![Documentation Status](https://readthedocs.org/projects/maniware/badge/?version=latest)](https://maniware.readthedocs.io/en/latest/?badge=latest)
| [**Reference docs**](https://maniware.readthedocs.io/en/latest/)
| [**Introduction**](#1-introduction)
| [**Architecture**](#2-architecture)
| [**Installation**](#3-requirements-and-installation)
| [**Case Studies**](#4-case-studies)


## 1. Introduction
**ManiWare** is an easy-to-use middleware that provides a team-level programming abstraction and the manipulator-level plugin mechanism for programming and building manipulator applications.

**ManiWare** provides a comprehensive set of functional components to allow developers to build cooperative manipulator applications.
Specifically, the package allows you to

- Design team-level cooperation mechanism with task scheduler and optimizer
- Perform manipulator-level motion control with robot models and controllers
- Run experiments on your customized manipulator teams
- Perform realistic simulations with [Pybullet](https://pybullet.org/wordpress/) simulation engine

## 2. Architecture
We propose the architecture and its underlying functional components, and the diagram are shown below:

<p style="text-align:center">
  <img src="docs/source/_static/maniware_arch.png" width="600" />
</p>


## 3. Requirements and Installation
**ManiWare** requires Python3 and Pybullet to be installed on your system.

Please refer to the [installation page](https://maniware.readthedocs.io/en/latest/installation/) for a more detailed installation guide.

To install the middleware, download the source code from
```
git clone https://github.com/sundyCoder/ManiWare .
```

## 4. Case Studies

We provide three case studies to show the basic features of **ManiWare**. Users can launch the applications directly.

### Case study 1: Cooperative pick-and-place

```
cd example
python3 exp_pick_place.py
```

### Case study 2: Cooperative object collection

```
cd example
python3 exp_collection.py
```

### Case study 3: Dynamic Reconfiguration

```
cd example
python3 exp_dyn_reconf.py
```

## 5. Documentation

For the detailed information, please refer to [ManiWare documentation](https://maniware.readthedocs.io/).


## 6. Citation

If you use this middleware or benchmark in your research, please cite the paper and the extended version
will be submitted to a Journal.

```
@inproceedings{cheng2022maniware,
  title={ManiWare: An Easy-to-Use Middleware for Cooperative Manipulator Teams},
  author={Cheng, Zhiqin and Cao, Jiannong and Chen, Jinlin},
  booktitle={2022 IEEE International Conference on Smart Computing (SMARTCOMP)},
  pages={349--355},
  year={2022},
  organization={IEEE}
}
```

## 7. License

This project is released under the [Apache 2.0 license](LICENSE).
