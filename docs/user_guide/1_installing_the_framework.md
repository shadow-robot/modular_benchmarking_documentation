# Installing the framework
Our software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions in [this section](#installing-the-framework) to get the latest Docker container up and running.

## Hardware specifications
In order to run our software and the ROS software stack you will need to meet some hardware requirements.
* CPU: Intel i5 or above
* RAM: 4GB or above Hard Drive: Fast HDD or SSD (Laptop HDD can be slow)
* Graphics Card: Nvidia GPU (optional)
* OS: Ubuntu 16.04 or 18.04 (Active development)

The most important one is to have a fast HDD or an SSD.

## Installing the framework
We have created a one-liner that is able to install Docker, download the image and create a new container for you. To use it, you first need to have a PC with Ubuntu installed on it (16.04 or 18.04 tested).

### Prerequisite
We **strongly** advise to run the one-liner on a machine without any version of docker installed. If you have never installed it, you can skip to the next subsection. If you are already using doccker, please be aware that the resulting container *might* not work. If it is the case, you can run the following lines:
```
sudo apt purge -y docker-engine docker docker.io docker-ce
sudo apt autoremove -y --purge docker-engine docker docker.io docker-ce
```
These instructions are uninstalling docker but should not remove any of the containers already stored on your machine.

### You have a nvidia card
If the machine you are going to use to run the framework has a Nvidia card **and the nvidia drivers are on**, then run the following line
```bash
bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e nvidia_docker=true tag=kinetic-nvidia-release reinstall=true sim_icon=false image=shadowrobot/modular_benchmarking_framework container_name=<container_name>
```
You can change `<container_name>` by the name you want to give to the container that you are going to use.

**If you have a nvidia card but are running on the xorg drivers, go to the other subsection!**

### You don't have a nvidia card
If you don't have a nvidia graphic card or if you do and don't use the nvidia drivers, please run
```bash
bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e nvidia_docker=false tag=kinetic-release reinstall=true sim_icon=false image=shadowrobot/modular_benchmarking_framework container_name=<container_name>
```
You can change `<container_name>` by the name you want to give to the container that you are going to use.

## Future releases
For now, the docker that you have downloaded contains Ubuntu 16.04 and ROS Kinetic. We are currently working on the release of the framework using Ubuntu 18.04 and ROS Melodic. Stay tuned!
