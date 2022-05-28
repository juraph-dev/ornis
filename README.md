# ORNIS (Open.Ros.Notcurses.Interface.System)


![Linux](https://img.shields.io/badge/-Linux-grey?logo=linux)
<!-- [![Travis Build Status](https://travis-ci.org/arnavb/cpp14-project-template.svg?branch=master)](https://travis-ci.org/arnavb/cpp14-project-template) -->
<!-- [![Appveyor Build Status](https://ci.appveyor.com/api/projects/status/qvt257817g7c66m9/branch/master?svg=true)](https://ci.appveyor.com/project/arnavb/cpp14-project-template/branch/master) -->
<!-- [![Coverity Scan Build Status](https://scan.coverity.com/projects/15312/badge.svg)](https://scan.coverity.com/projects/arnavb-cpp14-project-template) -->
<!-- [![codecov](https://codecov.io/gh/arnavb/cpp14-project-template/branch/master/graph/badge.svg)](https://codecov.io/gh/arnavb/cpp14-project-template) -->
<!-- [![Codacy Badge](https://api.codacy.com/project/badge/Grade/1c76469660ca411fa1f92ce0ef0c5cd3)](https://www.codacy.com/app/arnavb/cpp14-project-template?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=arnavb/cpp14-project-template&amp;utm_campaign=Badge_Grade) -->
<!-- [![Documentation](https://codedocs.xyz/arnavb/cpp14-project-template.svg)](https://codedocs.xyz/arnavb/cpp14-project-template/) -->



## What is it?

A Terminal User Interface for ROS2. 

## Current features
* View currently active topics/nodes/services
* Get information on currently active topics/nodes/services
* Support for service calls
* Streaming topics
* \*Kind of\* Supports mouse workflows

## Videos

## Installation
You're going to need Notcurses. You can either initialise it as a submodule to this repo, and compile alongside ORNIS, or you can install it [https://repology.org/project/notcurses/versions](using your package manager).
Ensure you have ROS2 (Foxy or newer) installed and sourced. 
``` sh
$git clone https://gitlab.com/juraph/ornis.git
$cd ornis
$git submodule init
$git submodule update
$cd tools && ./compile.sh
$cd ../../build/ornis
$./ornis
```

### Prerequisites
[Notcurses](https://github.com/dankamongmen/notcurses) \
[ROS2 Foxy or above](https://docs.ros.org/en/foxy/index.html) \
[Poco Foundation](https://pocoproject.org/) (This will most likely be changed very soon. I don't really need to be using it)

## Future roadmap


## Ackowledgements
[ros2_introspection](https://github.com/facontidavide/ros2_introspection) I blatantly stole a ton of code from here \
[dynamic message introspection tools](https://github.com/osrf/dynamic_message_introspection) And here \
[rosshow](https://github.com/dheera/rosshow) I actually started ORNIS _before_ I found rosshow (Promise), but once I saw it, it really fueled the fire, having shown me the potential of clever data visualisation. It's a pretty great piece of software. \
[Notcurses](https://github.com/dankamongmen/notcurses), the library powering the visuals of ORNIS. [Nick Black](https://github.com/dankamongmen) (The creator) is a super interesting dude, and I've enjoyed reading his [guidebook](https://nick-black.com/htp-notcurses.pdf), which is how I was introduced to Notcurses. \
[Arnav Borborah](https://github.com/arnavb/cpp14-project-template) Their cpp14 template is pretty great, allowed me to quickly get ORNIS up and running with barely any hassle. Having a template to look at how things like CI and test modules should be layed out was also extremely useful (Even though I don't use either, ha)

## Further reading
I wrote a long winded posted about [ORNIS on my blog](juraph.com/ornis/introducing_ornis) if you have absolutely nothing better to do with your time. 


<!-- ### Building the Code -->

<!-- #### Prerequisites -->
<!-- The following tools must be preinstalled before using this template: -->
<!-- - [`CMake`](https://cmake.org/install/) (At least v3.1): For building the code. -->
<!-- - [`Doxygen`](https://www.stack.nl/~dimitri/doxygen/manual/install.html): For generating documentation. -->


<!-- The documentation for this project (sample code and usage of this project) is hosted on [codedocs.xyz](https://codedocs.xyz/arnavb/cpp14-project-template/index.html). -->

<!-- Documentation about build targets, CMake options, the directory structure used, and documentation are all available in the link above. -->

<!-- ## License -->

<!-- ![CC0](http://i.creativecommons.org/p/zero/1.0/88x31.png) -->

<!-- To the extent possible under law, [Arnav Borborah](https://github.com/arnavb/cpp14-project-template) has waived all copyright and related or neighboring rights to C++14 Project Template. This work is published from: United States. -->

<!-- The above basically means that while you do not have to give me attribution for this template, it would be gladly appreciated! -->
