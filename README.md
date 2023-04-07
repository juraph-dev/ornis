# ORNIS (Open.Ros.Notcurses.Interface.System)

[![build status](https://gitlab.com/juraph/ornis/badges/master/pipeline.svg)](https://gitlab.com/juraph/ornis/-/commits/master)

## What is it?

A waste of time (A Terminal User Interface for ROS2 (a ROS2 TUI)).

## Current features
* View currently active topics/nodes/services
* Get information on currently active topics/nodes/services
* Support for service calls
* Streaming topics
* \*Kind of\* Supports mouse workflows

## Videos
Small demo video of topic selection and service calls.


https://user-images.githubusercontent.com/28330806/230512415-6c214e54-f59b-49c7-a2fa-0de811d996ee.mp4


### Prerequisites
[Notcurses](https://github.com/dankamongmen/notcurses) \
[ROS2 Foxy or above (Or below, I'm still working on figuring that out)](https://docs.ros.org/en/foxy/index.html) 

Windows and mac aren't currently supported. Mac might work, feel free to try it and let me know how it goes. 

## Installation
You're going to need Notcurses. You can either initialise it as a submodule to this repo, and compile alongside ORNIS, or you can install it [using your favourite package manager](https://repology.org/project/notcurses/versions).
Ensure you have ROS2 (Foxy or newer) installed and sourced.
 
Install notcurses deps
```sh
apt-get install build-essential cmake doctest-dev libavdevice-dev libdeflate-dev libgpm-dev libncurses-dev libqrcodegen-dev libswscale-dev libunistring-dev pandoc pkg-config
```
Build ornis
``` sh  
mkdir ornis_ws && cd ornis_ws
git clone https://gitlab.com/juraph/ornis.git 
cd ornis
git submodule update --init
cd tools && ./compile.sh
cd ../../build/ornis
./ornis
```

## Usage
Just run ./ornis, then trust the bar at the top that tells you how to use it. You can probably click things too, if you're one of those kinds of people. 


## Future roadmap
I still have a ton of things I'd like to implement. To name a few:

1. Proper topic visualisation, ala [rosshow](https://github.com/dheera/rosshow). At the moment, you only get the raw data displayed, I'd like to provide a meaningful and intuitive way to actually see the data being thrown around.
2. Action support. I don't personally use Actions frequently, so it wasn't a very high priority for me. It'll be pretty easy for me to add it, the hard part of ORNIS has already been layed out, I just need to get around to doing it.
3. Add support for multiple topic streams at once. The foundation is already there to allow the user to, for example: view a topic, while sending out a service call.
4. I want to add a "graph view" for all of the currently launched nodes. In the same vein as [Graphviz](https://graphviz.org/gallery/), with nodes linked by publishers/subscribers. I can see it being very useful (And fun) to be able to get an easy to digest visualisation of the node structure currently running on the robot.

## Ackowledgements
[ros2_introspection](https://github.com/facontidavide/ros2_introspection) I blatantly stole a ton of code from here \
[dynamic message introspection tools](https://github.com/osrf/dynamic_message_introspection) And here \
[rosshow](https://github.com/dheera/rosshow) I actually started ORNIS _before_ I found rosshow (Promise), but once I saw it, it really fueled the fire, having shown me the potential of clever data visualisation. It's a pretty great piece of software. \
[Notcurses](https://github.com/dankamongmen/notcurses), the library powering the visuals of ORNIS. [Nick Black](https://github.com/dankamongmen) (The creator) is a super interesting dude, and I've enjoyed reading his [guidebook](https://nick-black.com/htp-notcurses.pdf), which is how I was introduced to Notcurses. \
[Arnav Borborah](https://github.com/arnavb/cpp14-project-template) Their cpp14 template is pretty great, allowed me to quickly get ORNIS up and running with barely any hassle. Having a template to look at how things like CI and test modules should be layed out was also extremely useful (Even though I don't use either, ha)

## Further reading
I wrote a long winded posted about [ORNIS on my blog](juraph.com/ornis/introducing_ornis), for if you have absolutely nothing better to do with your time. 

