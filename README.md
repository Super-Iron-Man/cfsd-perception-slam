# state-estimation / SLAM

Visual Inertial SLAM

Sensors:
- camera
- IMU

How to build:

- Locally build

~~~bash
git clone https://github.com/chalmersfsd/cfsd-perception-slam.git
cd cfsd-perception-slam
git checkout develop # master branch is not updated as often as develop branch
mkdir build
cd build
cmake -D FOR_CFSD=ON -D FOR_EUROC=OFF -D WITH_VIEWER=ON -D WITH_IMSHOW=ON .. # turn either cfsd or euroc ON, and the other OFF; turn on WITH_VIEWER and WITH_IMSHOW if you want visulization
make # if want to speed up compiling, add flag -j2 or -j4 to use multi-thread compiling
~~~

- Docker

~~~bash
git clone https://github.com/chalmersfsd/cfsd-perception-slam.git
cd cfsd-perception-slam
git checkout develop # master branch is not updated as often as develop branch
docker build -f Dockerfile.amd64 -t slam . # if you need different config in cmake, change it in Dockerfile.amd64
~~~



How to use:

- For CFSD

  Prepare the .rec file, replay using *opendlv-vehicle-view* and *video-h264-decoder*

  Make sure in the config file **cfsd-perception-slam/config/cfsd.yml** the image resolution is right, and then run the slam program:

  ~~~bash
  cd cfsd-perception-slam/bin
  ./cfsd-state-estimation --cid=253 --name=img.argb --config=../config/cfsd.yml #--verbose
  ~~~



- For EUROC

  Download the dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets in *ASL Dataset Format*

  In the config file **cfsd-perception-slam/config/euroc.yml** change the dataset path

  ~~~bash
  cd cfsd-perception-slam/bin
  ./euroc-state-estimation ../config/euroc.yml
  ~~~

  

Current issue / problem:

- camera-IMU calibration for CFSD application

- robustness

- accuracy