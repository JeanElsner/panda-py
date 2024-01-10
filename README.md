<div align="center"><img alt="panda-py Logo" src="https://raw.githubusercontent.com/JeanElsner/panda-py/main/logo.jpg" /></div>

<h1 align="center">panda-py</h1>

<p align="center">
  <a href="https://github.com/JeanElsner/panda-py/actions/workflows/main.yml"><img alt="Continuous Integration" src="https://img.shields.io/github/actions/workflow/status/JeanElsner/panda-py/main.yml" /></a>
  <a href="https://github.com/JeanElsner/panda-py/blob/main/LICENSE"><img alt="Apache-2.0 License" src="https://img.shields.io/github/license/JeanElsner/panda-py" /></a>
  <a href="https://pypi.org/project/panda-python/"><img alt="PyPI - Published Version" src="https://img.shields.io/pypi/v/panda-python"></a>
  <img alt="PyPI - Python Version" src="https://img.shields.io/pypi/pyversions/panda-python">
  <a href="https://jeanelsner.github.io/panda-py"><img alt="Documentation" src="https://shields.io/badge/-Documentation-informational" /><a/>
</p>

Finally, Python bindings for the Panda. These will increase your productivity by 1000%, guaranteed[^1]!

## Getting started

To get started, check out the [tutorial paper](https://www.sciencedirect.com/science/article/pii/S2352711023002285), Jupyter [notebooks](https://github.com/JeanElsner/panda-py/tree/main/examples/notebooks) and other examples you can run directly on your robot. For more details on the API, please refer to the [documentation](https://jeanelsner.github.io/panda-py/).

## Extensions

* [dm_robotics_panda](https://github.com/JeanElsner/dm_robotics_panda) Panda model and tools for reinforcement learning in `dm_robotics`

## Install

```
pip install panda-python
```

This will install panda-py and all its requirements. The pip version ships with libfranka 0.9.2, the newest version for the Franka Emika Robot. Please refer to the section below if you use an older system version or the more recent Franka Research 3 robot. 

## libfranka Version

There are currently two robot models available from Franka Emika: the Franka Emika Robot (FER, formerly known as Panda) and the Franka Research 3 (FR3). Depending on the installed firmware, the FER supports version <0.10 while the FR3 requires version >=0.10. For details, refer to [this](https://frankaemika.github.io/docs/compatibility.html) compatibility matrix. If you need a libfranka version different from the default 0.9.2, download the respective zip archive from the [release page](https://github.com/JeanElsner/panda-py/releases). Extract the archive and install the wheel for your Python version with pip, e.g., run
```
pip install panda_python-*libfranka.0.7.1-cp310*.whl
```
to install the binary wheel for libfranka 0.7.1 and Python 3.10.

# Citation

If you use panda-py in published research, please consider citing the [original software paper](https://www.sciencedirect.com/science/article/pii/S2352711023002285).

```
@article{elsner2023taming,
title = {Taming the Panda with Python: A powerful duo for seamless robotics programming and integration},
journal = {SoftwareX},
volume = {24},
pages = {101532},
year = {2023},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2023.101532},
url = {https://www.sciencedirect.com/science/article/pii/S2352711023002285},
author = {Jean Elsner}
}
```

[^1]: Not actually guaranteed. Based on a sample size of one.
