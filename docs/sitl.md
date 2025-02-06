# PX4 Software in the Loop Simulation for DEXI Development

### 1. Clone DEXI Repo

```
git clone -b develop https://github.com/droneblocks/dexi --recurse-submodules
```

### 2. Launch PX4 SITL in Docker

Fire up a Docker container and map the DEXI repo from the host into the container. Make sure to cd directory where you cloned the DEXI repo.

```
cd ~/Desktop/dexi
```

> **_NOTE:_** Depending on your computer system's architecture we've provided two variants of the Docker image. Generally speaking, arm64 is for Mac computers with Apple silicon and amd64 is for Windows based PCs. But there is always the exception. If you are uncertain or have any questions please feel free to jump into our [Discord](https://discord.gg/Wjw7wGf7Wn).

For arm64:
```
docker run -p 6080:80 --security-opt seccomp=unconfined -v ${PWD}:/dexi_ws/src/ --name dexi-dev --shm-size=512m droneblocks/dexi-px4-sitl-arm64:1.15
```

For amd64:
```
docker run -p 6080:80 --security-opt seccomp=unconfined -v ${PWD}:/dexi_ws/src/ --name dexi-dev --shm-size=512m droneblocks/dexi-px4-sitl-amd64:1.15
```

Pay attention to the -v argument that handles mapping the cloned dexi repo into the /dexi_ws/src folder inside the container. This enables us to make changes on the host using VS Code and those changes will be available inside the container.

This volume mapping also has the benefit of your changes not being deleted when the container is destroyed.

### 3. Web Access

Your container is now accessible directly via web browser! Go to https://localhost:6080 to access it.

### 4. Run SITL

Double-click the Terminator icon on the desktop. You will be in the /px4 directory by default. To ensure communication between the Docker container and QGroundControl running on your host (Desktop machine) you will need to run a script:

```
./edit_rcS.bash
```

Once that is done you will change into the PX4-Autopilot directory:

```
cd PX4-Autopilot
```

and run SITL with the following command:

```
make px4_sitl gz_x500
```

After a few moments you will see a drone loaded into the Gazebo simulation environment

### 5. Connect QGroundControl

Open QGC on your desktop machine and it should automatically connect to PX4 SITL running in Docker. Tada! Play around with your simulated drone in QGC.
