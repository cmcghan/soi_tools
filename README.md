# License

*OpenUxAS* is developed by the Air Force Research Laboratory, Aerospace System Directorate, Power and Control Division. 
The LMCP specification and all source code for *OpenUxAS* is publicaly released under the Air Force Open Source Agreement
Version 1.0. See LICENSE.md for complete details. The Air Force Open Source Agreement closely follows the NASA Open Source
Agreement Verion 1.3. **NOTE the terms of the license include registering use of the software by emailing <a href="mailto:afrl.rq.opensource@us.af.mil?subject=OpenUxAS Registration&body=Please register me for use of OpenUxAS. Name: ____________">afrl.rq.opensource@us.af.mil</a>.**

*soi_tools* was forked and modified from the https://github.com/backesj/soi_tools repository.
The main LICENSE.md file for soi_tools should be the same as the OpenUxAS LICENSE.

Modifications by Catharine McGhan are Copyright 2017 University of Cincinnati. See file headers.


# *** WARNING!!! ***

Note that THIS CODE IS NOT YET READY FOR PRIMETIME!!! This is very much still in-progress code.

Connection to (1) a Gazebo turtlebot model and (2) a Gazebo UAV model is coming soon!

Complete instructions for use are coming soon!


# POLYMORPHISM AHOY! -- HERE THERE BE DRAGONS!!

Note that UxAS relies upon (as in: -expects-) downcasting to occur for many of its variables. It uses dynamic messaging. This means that the current set of static ROS messages that are created by `./lmcp2rosmsg/lmcp2rosmsg.py` from the OpenUxAS MDMs files are meant to be used AS A REFERENCE BY ROS USERS currently, to give an idea of the expected internal data and datatypes. (You can `catkin_make` the ROS packages, but they will not "work" the way you think.) They're meant to be something of a one-stop at-a-glance reference for UxAS message data. See the comments in the .msg files if you're interested.


# Supported Operating Systems

For an Ubuntu 16.04 or Mac OS X system with the listed prerequisite tools installed, UxAS should build from source without issue. The soi_tools and rss_git_lite python scripts should function as-expected on Ubuntu Linux, but I make no claims as to Mac OS X; they likely will not work under Windows.


# Configure UxAS and Related Projects

Expected file system layout:
```
/home/$USER/UxAS_$USER
                      /OpenAMASE
                                /OpenAMASE
                                          /config
                                          /data
                                          /dist
                                               OpenAMASE.jar <-- add this here to avoid compilation
                                          /docs
                                          /example scenarios
                                          /lib
                                          /native
                                          /nbproject
                                          /run
                                          /src
                      /LmcpGen
                              /dist
                                   LmcpGen.jar <-- add this here to avoid compilation
                              /nbproject
                              /src
                      /OpenUxAS
                               /3rd
                               /doc
                               /examples
                                        /02_Example_WaterwaySearch
                                                                  /ext_logger
                               /mdms
                               /resources
                               /src
                               /tests
                               /wrap_patches
                      /soi_tools
                                /lmcp2ros
                      /rss_git_lite
                                   /common
```


# Installing Prerequisites + Dependencies and Getting the Code on Ubuntu Linux (Partially-Automated)

1. Install cmcghan/OpenUxAS and other packages to /home/$USER/UxAS_$USER (see: https://github.com/cmcghan/OpenUxAS/blob/develop/README.md)
   - `mkdir /home/$USER/UxAS_$USER`
   - `cd /home/$USER/UxAS_$USER`
   - `git clone https://github.com/cmcghan/OpenUxAS.git`
   - `cd OpenUxAS`
   - `./install_most_deps.sh`
   - `./checkout_plus_config.sh -c /home/$USER/UxAS_$USER devel`
1. Add soi_tools and rss_git_lite for ros_adapter.py use
   - `cd /home/$USER/UxAS_$USER`
   - `git clone https://github.com/cmcghan/soi_tools.git`
   - `git clone https://github.com/cmcghan/rss_git_lite.git`


# Getting the Code and Running the Examples (WORK-IN-PROGRESS!)

1. Install cmcghan/OpenUxAS and other packages to /home/$USER/UxAS_$USER (see: https://github.com/cmcghan/OpenUxAS/blob/develop/README.md)
   - `mkdir /home/$USER/UxAS_$USER`
   - `cd /home/$USER/UxAS_$USER`
   - `git clone https://github.com/cmcghan/OpenUxAS.git`
   - `cd OpenUxAS`
   - `./install_most_deps.sh`
   - `./checkout_plus_config.sh -c /home/$USER/UxAS_$USER develop`
1. Add soi_tools and rss_git_lite for ext_logger use
   - `cd /home/$USER/UxAS_$USER`
   - `git clone https://github.com/cmcghan/soi_tools.git`
   - `git clone https://github.com/cmcghan/rss_git_lite.git`
1. ???
   - ???
1. Run examples
   - Example 2: See README.md in `examples/02_Example_WaterwaySearch` for the base case
   - ???
