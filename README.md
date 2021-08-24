# Jays--

HOW TO CONNECT PC TO ROBOT THROUGH SSH
 - Find Robot's IP by typing $ifconfig
 - On PC, type $ssh username@IP   e.g. $ssh erion@192.168.0.24

HOW TO SHARE FOLDER BETWEEN PC AND ROBOT
 - Install Samba  $sudo apt install samba
 - Open Samba config file.   $sudo nano /etc/samba/smb.conf     **nano = text editor program
 - Change wins suuport to "yes"
 - And at the end of the conf file, setup the folder you want to share
e.g. in my case, catkin_ws_src folder.
[catkin_ws_src]
commnet = Samba on Ubuntu
path = /home/erion/catkin_ws/src/
read only = no
browsable = yes

 - Set up the Samba Password.   $sudo smbpasswd -a erion
 - Reboot  $sudo reboot



HOW TO INSTALL ROS PACKAGE
 - Navigate into the ROS source folder.    $roscd
 - $cd src
 - Install package by copying github repository.   $git clone https://github.com/...
 - Go back to catkin folder $cd ..
 - Build to compile the packages and build the messages. $catkin_make
 - source the development environment $source devel/setup.bash


Useful ROS commands:

$ roscd     = Navigate into the source folder

