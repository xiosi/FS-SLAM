# DO-SLAM

## build:

### build the whole project ( inclouding binary loading tools ):

build with library

```bash
     cd YourDirectory/DO-SLAM
     chmod +x build.sh
     ./build.sh
```

### only build

```bash
    cd YourDirectory/DO-SLAM
    mkdir build
    cd build
    cmake ..
    make -j
```

## Run:

```bash
    ./run/rgbd_tum config/ORB.txt path_to_settings path_to_sequence path_to_association
```
 <img src="./RUN.png" width = "300" height = "200" alt="图片名称" align=left />

<img src="./map.png" width = "300" height = "200" alt="图片名称" align=center />
## 数据集
 <img src="./fr3-lc-ape.png" width = "300" height = "300" alt="图片名称" align=left />

<img src="./room-ape.png" width = "300" height = "300" alt="图片名称" align=center />