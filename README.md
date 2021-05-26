##Project for multi-purpose robot control

A project for mobile robot control, a robot pursues the cube which could be controlled by mouse cursor, simply start the appropriate scene in Uniti project and enjoy.

Options for controlling the robot

Each option should be **in advance** selected in `Start()` method of `Assets/Scripts/UnityDifferentialBaseSimulation.cs`

- OpenCV-powered, go to `Scripts/VideoCapt.py`, first start the scene in Unity and then start the python script (with a bif of delay)
- Two self-contained methods in Unity, could be found in `Assets/Scripts/UnityDifferentialBaseSimulation.cs`


###Dependencies

---
- Unity3D (with maybe NuGet plugin)
- Python3 + `Scripts/requirements.txt`