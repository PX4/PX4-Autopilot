# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure(2) do |config|

  # Every Vagrant development environment requires a box. You can search for
  # boxes at https://atlas.hashicorp.com/search.
  config.vm.box = "ubuntu/trusty64"

  # Disable automatic box update checking. If you disable this, then
  # boxes will only be checked for updates when the user runs
  # `vagrant box outdated`. This is not recommended.
  # config.vm.box_check_update = false

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine. In the example below,
  # accessing "localhost:8080" will access port 80 on the guest machine.
  # MAVLink telemetry via UDP in SITL mode
  config.vm.network "forwarded_port", guest: 14556, host: 14556, protocol: "udp"
  # SITL simulation data
  config.vm.network "forwarded_port", guest: 14560, host: 14560, protocol: "udp"

  # Create a private network, which allows host-only access to the machine
  # using a specific IP.
  # config.vm.network "private_network", ip: "192.168.33.10"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  # config.vm.network "public_network"

  # Share an additional folder to the guest VM. The first argument is
  # the path on the host to the actual folder. The second argument is
  # the path on the guest to mount the folder. And the optional third
  # argument is a set of non-required options.
  config.vm.synced_folder ".", "/Firmware"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI when booting the machine
    vb.gui = false
    vb.customize ["modifyvm", :id, "--ioapic", "on"]
    vb.customize ["modifyvm", :id, "--cpus", "2"]

    # Since make and other tools freak out if they see timestamps
    # from the future and we share directories, tightly lock the host and guest clocks together (clock sync if more than 2 seconds off)
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-threshold", 2000]
    # Do this on start and restore
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-start"]
    vb.customize ["guestproperty", "set", :id, "/VirtualBox/GuestAdd/VBoxService/--timesync-set-on-restore", "1"]

    # Customize the amount of memory on the VM:
    vb.memory = "2048"
  end
  #
  # View the documentation for the provider you are using for more
  # information on available options.

  # Define a Vagrant Push strategy for pushing to Atlas. Other push strategies
  # such as FTP and Heroku are also available. See the documentation at
  # https://docs.vagrantup.com/v2/push/atlas.html for more information.
  # config.push.define "atlas" do |push|
  #   push.app = "YOUR_ATLAS_USERNAME/YOUR_APPLICATION_NAME"
  # end

  # Enable provisioning with a shell script. Additional provisioners such as
  # Puppet, Chef, Ansible, Salt, and Docker are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", privileged: false, inline: <<-SHELL
    # Ensure we start in the Firmware folder
    echo "cd /Firmware" >> ~/.bashrc
    # Install software
    sudo apt-get update
    sudo apt-get install -y build-essential ccache cmake clang-3.5 lldb-3.5 g++-4.8 gcc-4.8 genromfs libc6-i386 libncurses5-dev python-argparse python-empy python-serial s3cmd texinfo zlib1g-dev git-core zip
    pushd .
    cd ~
    wget -q https://launchpadlibrarian.net/186124160/gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2
    tar -jxf gcc-arm-none-eabi-4_8-2014q3-20140805-linux.tar.bz2
    exportline="export PATH=$HOME/gcc-arm-none-eabi-4_8-2014q3/bin:\$PATH"
    if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
    . ~/.profile
    popd
    # setup ccache
    mkdir -p ~/bin
    ln -s /usr/bin/ccache ~/bin/arm-none-eabi-g++
    ln -s /usr/bin/ccache ~/bin/arm-none-eabi-gcc
    ln -s /usr/bin/ccache ~/bin/g++-4.8
    ln -s /usr/bin/ccache ~/bin/gcc-4.8
    export PATH=~/bin:$PATH

    # Configure hardware related bits
    sudo apt-get -y remove modemmanager
    sudo usermod -a -G dialout $USER
  SHELL
end
