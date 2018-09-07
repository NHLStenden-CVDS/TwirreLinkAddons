Part of the Twirre architecture: <https://nhlstenden-cvds.github.io/Twirre>

# TwirreLinkAddons

* [General info](#general-info)
* [Dependencies](#dependencies)
* [Modules](#modules)
* [Building on Linux](#building-on-linux)
* [Building on Windows](#building-on-windows)
* [License](#license)
* [Examples](#examples)


## General info
This project provides some additional device interfaces for TwirreLink. The project contains several different 'modules' for different types of sensors.


## Dependencies
The TwirreLink Addons depend on the base [Twirrelink library][twirrelink-prj]. 


## Modules

* [myAHRS+](#myahrs)
* [gps](#gps)

### myAHRS+
The myARHS+ module provides support for adding a [myAHRS+ sensor][myahrs] to TwirreLink using an USB connection.

### GPS
The GPS module provides TwirreLink capabilities for two specific RTK GPS sensors: [SwiftNav Piksi][piksi] and [Tersus Precis BX305][precis].


## Building on Linux
TwirreLink Addons require a C++14-capable compiler to be built. An Eclipse CDT project file is included for each module. This is the recommended method of building modules:

* Import project into Eclipse (File > Import > Existing Projects into Workspace)
* Select correct build profile in eclipse (ARM or x86) -> (Right-click project > Build Configurations > Set active)
* Build project

This will produce a static library file (.a), which can be used in another project.


## Building on Windows
No project or makefiles are provided, but both the myAHRS+ and GPS modules should be Windows-compatible if you manually add the source files to a project.


## License
TwirreLinkAddons have been made available under the MIT license (see **LICENSE**).


## Examples
Examples to be added at a later date



[twirrelink-prj]: https://github.com/NHLStenden-CVDS/TwirreLink
[myahrs]: https://github.com/withrobot/myAHRS_plus
[piksi]: https://www.kickstarter.com/projects/swiftnav/piksi-the-rtk-gps-receiver
[precis]: https://www.tersus-gnss.com/product/bx305-oem-board
