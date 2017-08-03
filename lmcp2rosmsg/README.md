# *** WARNING!!! ***

Note that THIS CODE IS NOT COMPLETLY READY FOR PRIMETIME!!! This is very much still in-progress code.

What is currently missing is a ROS .msg --> LmcpGen MDMs converter. (We have a static converter here for the other way around.)

Complete instructions for use are coming soon!


# POLYMORPHISM AHOY! -- HERE THERE BE DRAGONS!!

Note that UxAS relies upon (as in: -expects-) downcasting to occur for many of its variables. It uses dynamic messaging. This means that the current set of static ROS messages that are created by `./lmcp2rosmsg/lmcp2rosmsg.py` from the OpenUxAS MDMs files are meant to be used AS A REFERENCE BY ROS USERS currently, to give an idea of the expected internal data and datatypes. (You can `catkin_make` the ROS packages, but they will not "work" the way you think.) They're meant to be something of a one-stop at-a-glance reference for UxAS message data. See the comments in the .msg files if you're interested.
