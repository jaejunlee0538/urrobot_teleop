# How to run
### send source code to Control Box
```shell
$ sh send_program.sh
```

### build test on PC
```shell
$ scons control_box=0
```

### build on Control Box
```shell
$ scons
or
$ scons control_box=1
```

### run
With real robot
```
$ ./build/real/main
```

Without robot(only printing commands)
```
$ ./build/sim/main
```


author : jaejunlee
