#Das image was als grundlage verwendet wird
FROM ros:noetic
#WIr erstellen den workspace und package damit wir darin arbeiten können
RUN mkdir -p /home/adrian/workspace/src/musclework_standalone
WORKDIR /home/adrian/workspace/src/musclework_standalone

#Hier kopieren wie das package rein.
COPY src/ src/
COPY launch/ launch/
COPY package.xml .
COPY CMakeLists.txt .

#Wie setzten das neue workdir auf workspace damit wir catkin ausfhren können
WORKDIR /home/adrian/workspace/


#ALs erstees source Punkt = source, dann apt get upade damit rosdep die dependen zies instalieren kann. (was macht das dahinter)
#Dann catkin_make
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; apt-get update; rosdep install --from-paths . --ignore-src -r -y; catkin_make'

#Erst git installieren dmit Pip das packgage von git instalieren kann. Anschließend die Apt-get update wieder entfernen(sanaty befehl)
RUN /bin/bash -c 'apt-get install -y git; python3 -m pip install git+https://github.com/dtolpin/bocd.git; rm -rf /var/lib/apt/lists/*'

# launch ros package
ENTRYPOINT /bin/bash -c 'source ./devel/setup.bash; roslaunch musclework_standalone musclework.launch'