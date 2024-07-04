#/bin/bash
pybind11-stubgen -o src --enum-class-locations RealtimeConfig:panda_py.libfranka --enum-class-locations VacuumGripperProductionSetupProfile:panda_py.libfranka --numpy-array-use-type-var panda_py
