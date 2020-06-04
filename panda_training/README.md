# Panda_training

This package contains several examples of RL training scripts that can be used together
with the simulation and Openai gym environments of the `panda_openai_sim` package.

## How to use

In order to use the scripts in this package you first must build the `panda_openai_sim`
package. Please see the [online docs](https://rickstaa.github.io/panda_openai_sim/) for
installation and usage instructions. After you have successfully build the `panda_openai_sim`
package you can start the Panda openai simulation using the following ROS command:

```bash
roslaunch panda_openai_sim start.launch
```

After the simulation is running you can run any of the example training scripts that are
contained in the scripts folder.

## References

The following repositories were used for creating the example training scripts:

- [Reinforcement-learning-with-tensorflow](https://github.com/MorvanZhou/Reinforcement-learning-with-tensorflow)
- [Stable-baselines](https://stable-baselines.readthedocs.io/en/master/)
