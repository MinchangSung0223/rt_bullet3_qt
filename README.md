# bullet3_NoGUI_cmake_template
# bullet3 install
먼저 bullet3를 build해야 합니다. 
임의의 폴더에 bullet3를 내려받고 다음과정대로 build를 진행합니다.
```bash
  git clone https://github.com/bulletphysics/bullet3.git
  cd bullet3  
  echo 'export BULLET_PHYSICS_SOURCE_DIR=' echo ${PWD} >> ~/.bashrc
  ./build_cmake_pybullet_double.sh
  cd build_cmake
  cmake  -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..
  make -j$(nproc)
  sudo make install
  cd local_install/lib
  echo 'export BULLET_PHYSICS_LIB=' echo ${PWD} >> ~/.bashrc
```

# template build
BULLET_PHYSICS_SOURCE_DIR, BULLET_PHYSICS_LIB가 설정된 상태에서 
다음의 명령어로 빌드를 진행합니다.
예제는 GUI없이 plane.urdf를 불러오고 1000Hz로 stepSimulation을 진행합니다.

```bash
git clone https://github.com/MinchangSung0223/bullet3_NoGUI_cmake_template.git
cd bullet3_NoGUI_cmake_template
mkdir build
cd build
cmake .. -DBULLET_PHYSICS_SOURCE_DIR=${BULLET_PHYSICS_SOURCE_DIR} -DBULLET_PHYSICS_LIB=${BULLET_PHYSICS_LIB}
make j$(nproc)
./hello
```


# 문제점
make install할 경우
```
/usr/local/lib
``
위에 install됨, 그런데 ros2나 ros가 설치되어 있는경우

```
/usr/lib/x86_64-linux-gnu
```
위 폴더에 libBullet*.so.2.88 들이 설치되어 있음

따라서 충돌을 막기 위해 다음과 같이 설정
```
sudo cp  /usr/local/lib/libBullet* /usr/lib/x86_64-linux-gnu
```

