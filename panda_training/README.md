# Panda_training

This package contains several examples of RL training scripts that can be used together
with the simulation and Openai gym environments of the `panda_openai_sim` package.

## How to use

To use the scripts in this package, you first must build the `panda_openai_sim`
package. Please see the [online docs](https://rickstaa.github.io/panda_openai_sim/) for
installation and usage instructions. After you have successfully built the `panda_openai_sim` package, you can start the Panda Openai simulation using the following ROS command:

```bash
roslaunch panda_openai_sim start.launch
```

After the simulation is running, please install the 'panda_training' python package using
the `pip install .` command inside the `panda_training` folder. After the python package
is installed, you can then run any of the example training scripts that are
contained in the `scripts` folder inside a [python3 environment](https://rickstaa.github.io/panda_openai_sim/get_started/install.html#py3-virtual-env).

## References

The following repositories were used for creating the example training scripts:

- [Reinforcement-learning-with-tensorflow](https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow)
- [Stable-baselines](https://stable-baselines.readthedocs.io/en/master/)
