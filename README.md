# rl2023_ex2_P38000151

To launch all the nodes :

```
$ roslaunch ex2_custom_msg rl2023_ex2.launch
```

A leaky integrator is used to filter the signal. The a parameter can be used to calibrate the smoothing of the input signal (it can be
changed in the sin_param.yaml in /conf)
