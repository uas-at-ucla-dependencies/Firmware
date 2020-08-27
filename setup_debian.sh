cd "$(dirname "$0")"

git submodule update --init --recursive

# Setup script provided by PX4
source ./Tools/setup/ubuntu.sh

# Install python3-venv for Python virtual environment
sudo apt-get install -y python3-venv

# Needed to compile sitl gazebo code
sudo apt-get install -y libopencv-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libboost-all-dev
sudo apt-get install -y protobuf-compiler

# Install gazebo for Raspberry Pi (running debian stretch)
if [ "$(lsb_release -is)" == "Raspbian" ]; then
  if [ "$(lsb_release -cs)" == "stretch" ]; then
    # Only version 7 is available on Raspbian (and only on stretch)
    sudo apt-get install -y gazebo7

    # Needed to compile sitl gazebo code
    sudo apt-get install -y libgazebo7-dev

    sudo apt-get install -y libatlas-base-dev # Needed if installing numpy via pip3

    # Increase swap space, since the Raspberry Pi Zero doesn't have enough RAM for building
    echo "" | sudo tee -a /etc/dphys-swapfile
    echo "# Added by px4-sitl-wrapper. Needed for building (not for running) the SITL code" | sudo tee -a /etc/dphys-swapfile
    echo "CONF_SWAPSIZE=1024" | sudo tee -a /etc/dphys-swapfile
    sudo /etc/init.d/dphys-swapfile restart
  fi
else # Probably Ubuntu
  # Needed to compile sitl gazebo code
  sudo apt-get install -y libgazebo9-dev # version 9 should be available on Ubuntu
fi
