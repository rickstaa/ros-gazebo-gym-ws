# Create a virutal env

You are recommended to use the openai\_ros package inside a virtual env. This virtual env can be created using the following command:

```python
python -m venv ./openai_ros --system-site-packages
```

In this this the  `--system-site-packages` flag makes sure that the virtual environment has acces to the system site-packages. Alternatively you can also use the [RoboStack ros-noetic](https://github.com/RoboStack/ros-noetic) [conda-forge](https://conda-forge.org/) packages (see this [blog post](https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3) for more information).
