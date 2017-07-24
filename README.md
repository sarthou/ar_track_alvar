# <img src="readme_images/MarkerData_0.png" width="50"> ar_track_alvar
Original package
- Package maintained by Scott Niekum
- sniekum@willowgarage.com
- sniekum@cs.umass.edu

Current package
- Package maintained by Guillaume Sarthou
- gsarthou@laas.fr

This repository is a fork from sniekum/ar_track_alvar. This repository offers two nodes to easily create tags.
This readme will only show how to use the new nodes, for more information on ar_track_alvar, see http://www.ros.org/wiki/ar_track_alvar.

## Tags preamble

We always consider the orientation shown below as the reference orientation of the mark. You can distinguish this orientation with the two empty squares represented by the red box on this example.

<img src="readme_images/MarkerRef.png" width="100">

When you create a new tag, an XML file will be generated with it. On this file you will find the description of the corners of each tag like this one:
```
    <corner x="-2" y="-2" z="0" />
    <corner x="2" y="-2" z="0" />
    <corner x="2" y="2" z="0" />
    <corner x="-2" y="2" z="0" />
```
If you want to manually modify this file, assume that the corners are as follows.

<img src="readme_images/corners.png" width="100">

Finally, we will consider the following 3D marker for each tag.

<img src="readme_images/MarkerAxes.png" width="100">

## Flag generation
A tag called "flag," is a single 2D tag that refers to a position offset to its own.

### Usage
```
$ rosrun ar_track_alvar createFlag
```
First, enter the last ID you are using. Be careful, all the tags (flag or not), **must have a different identifier**, that is why the node automatically increases the identifiers of tags.

The reference unit use on this node is the centimeter.
After specifying the size of the tag, you can describe the position offset on X, Y, and Z.

<img src="readme_images/MarkerFlagExample.png" width="150">

### Tips
- Always take 1 cm margin on the size of your tag to keep a white border for better detection.
- Be carefull about the tag orientation

## Cube generation
A tag called "Cube" is composed of 6 tags assembled in a cube. Only the master tag will be displayed regardless of the orientation of the cube.

### Usage
```
$ rosrun ar_track_alvar createCube
```
First, enter the last ID you are using. Be careful, all the tags (flag or not), **must have a different identifier**, that is why the node automatically increases the identifiers of tags (**+6 per cube**).

The reference unit use on this node is the centimeter.
After specifying the size of the tag, specified the size of the cube and ... this is all you need to do !!

The node will generate a pattern that you have to cut and stick.
### Tips
- Always take 1 cm margin on the size of your tag to keep a white border for better detection.
- If you have to cut all the tags of the pattern, make sure to stick them with the correct order and orientation.
