This is a Gazebo model plugin that uses the magnetic dipole-dipole model to compute the force and torque between multiple magnets. The plugin is enabled per model and looks for other models in the gazebo world that have the same plugin. It only simulates magnetic interactions between magnets (not other materials).

The plugin requires the `bodyName` tag specifying which link is the actual magnet and `dipole_moment` which is a vector specifying the dipole moment of said magnet.

Example:

      <plugin name="dipole_magnet" filename="libgazebo_dipole_magnet.so">
        <bodyName>magnet</bodyName>
        <dipole_moment>0 0 1.26</dipole_moment>
      </plugin>

The magnitude of dipole moment of a cylindrical magnet can be computed using the formula:

        dm_mag = B_max * 4*pi * (h/2)^3/(2*mu_0) 

Where `B_max` is the remanence of the magnet, `h` is the height and `mu_0=4*pi*1e-7` is the permeability constant.


## Building the plugin

The plugin is a ros package so the build process is the same as any other package.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/vustormlab/storm_gazebo_ros_magnet.git
$ catkin_make -C ~/catkin_ws
```

## Running Example

To run the example in the worlds/ directory run

```
$ rosrun gazebo_ros gazebo dipole_magnet.world
```

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Lw2KfwgySWI" target="_blank"><img src="http://img.youtube.com/vi/Lw2KfwgySWI/0.jpg" 
alt="Magnet simulation in Gazebo" width="480" height="360" border="10" /></a>

## Citing This Work

If you use this work, please cite our [RSS 2016 paper](http://www.roboticsproceedings.org/rss12/p18.html):

```
@INPROCEEDINGS{Taddese-RSS-16, 
    AUTHOR    = {Addisu Z. Taddese AND Piotr R. Slawinski AND Keith L. Obstein AND Pietro Valdastri}, 
    TITLE     = {Closed Loop Control of a Tethered Magnetic Capsule Endoscope}, 
    BOOKTITLE = {Proceedings of Robotics: Science and Systems}, 
    YEAR      = {2016}, 
    ADDRESS   = {AnnArbor, Michigan}, 
    MONTH     = {June}, 
    DOI       = {10.15607/RSS.2016.XII.018} 
} 
```
