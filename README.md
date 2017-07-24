## <img src="readme_images/MarkerData_0.png" width="50">
# ar_track_alvar
- Package maintained by Scott Niekum
- sniekum@willowgarage.com
- sniekum@cs.umass.edu

This repository is a fork from sniekum/ar_track_alvar. This repository offers two nodes to easily create tags.
This readme will only show how to use the new nodes, for more information on ar_track_alvar, see http://www.ros.org/wiki/ar_track_alvar.

## Tags preamble

We always consider the orientation shown below as the reference orientation of the mark. You can distinguish this orientation with the two empty squares represented by the red box on this example.
## <img src="readme_images/MarkerRef.png" width="100">

When you create a new tag, an XML file will be generated with it. On this file you will find the description of the corners of each tag like this one:
'''
    <corner x="-2" y="-2" z="0" />
    <corner x="2" y="-2" z="0" />
    <corner x="2" y="2" z="0" />
    <corner x="-2" y="2" z="0" />
'''
If you want to manually modify this file, assume that the corners are as follows.
## <img src="readme_images/corners.png" width="100">

Finally, we will consider the following 3D marker for each tag.
## <img src="readme_images/MarkerAxes.png" width="100">

## Flag generation


## Cube generation
