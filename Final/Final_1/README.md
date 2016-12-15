# GPView

Adarsh's GPU framework, with his and Iddo's respective Hausdorff algorithms implemented

# Author(s)

* Adarsh Krishnamurty (adarsh@iastate.edu)

# Tested with

* ACIS 3D R26
* CUDA 7.5
* Cg 3.1
* Freeglut 3.0.0
* GLEW 1.13.0

# How to run GPView

## Required installations

* ACIS 3D R26 64-bit (Check \\ideafs.me.iastate.edu\Software\ network share to download)
* nVidia's [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit).
* nVidia's [Cg Toolkit](https://developer.nvidia.com/cg-toolkit). Don't forget to install 64-bit libraries during Cg installation.
* [Windows 10 SDK](https://dev.windows.com/en-us/downloads/windows-10-sdk) for DirectX and OpenGL libraries.
* [GLEW](http://glew.sourceforge.net/) Windows binaries
* [Freeglut](http://www.transmissionzero.co.uk/software/freeglut-devel/) MSVC libraries.
 
## Setup

* Extract GLEW and Freeglut in a directory (e.g. D:\Libraries\). The directory structure will be "D:\Libraries\glew-1.13.0" and "D:\Libraries\freeglut"
* Add these into PATH variable. You can use command line or follow

1. This PC (right click) and Properties
2. Advanced System Settings (on the left menu)
3. Environmental Variables
4. Under System Variables find **Path** and add the following: "D:\Libraries\glew-1.13.0\bin\Release\x64" and "D:\Libraries\freeglut\bin\x64" If you extracted them in a different directory, change them accordingly. 

* Also add ACIS to the PATH. Default ACIS 64-bit installation is located under `C:\Program Files\Spatial\acisR26\NT_VC12_64_DLLD\code\bin\`
* Create a new system variable, name it as **A3DT** and assign it to `C:\Program Files\Spatial\acisR26`
* **Optional Step (might be required for other programs using ACIS 3D):** Create another new system variable, name it as **ARCH** and assign it to `NT_VC12_64_DLLD`. The **D** letter at the end basically means **DEBUG MODE**. Use `NT_VC12_64_DLL` for release versions.

## Configuring CUDA parameters

* The computers you use probably have different GPUs. In order to run GPView as fast as possible, it is necessary to use correct CUDA compile capability. You can check from nVidia's website or [Wikipedia](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs).
* In Visual Studio 2013, it is easy to change CUDA compile capability. First, right click on GPView project (not the solution).
* Under *Configuration Properties*, find CUDA C/C++ and go to Device section.
* Change *Code Generation* variable according to your GPU.
* For Quadro K2200, use **computer_50,sm_50** value. In this value, **50** means **compile capability version 5.0**. Check [nVidia's website](https://developer.nvidia.com/cuda-gpus) for details.

## Compiling and running

* Open the project with Visual Studio 2013
* Locate Solution Explorer window. If you can't find it, press CTRL + ALT + L.
* Right click on **Solution 'GPView'** (at the top) and choose **Build Solution**
* If necessary, choose GPView as the starting project (right click on GPView, you will see it inside the menu)
* You might need to run GPView without debugger attached to it.
* **Don't forget to choose x64 build configuration from the dropdown menu (VERY IMPORTANT!)**. Note that, if you want to use Win32 (a.k.a. 32-bit) option, you need to install and set PATH for 32-bit versions of the libraries in consideration.

## In case of problems...

* Check system variables pointing the right directories.
* Check all the libraries are installed
* If you change or add **any system or user variable** (e.g. PATH), restart Visual Studio 2013.
* In case of linker errors related to ACIS, check you are compiling for x64 from the output screen or Debugger toolbar.
* If you are getting "File not specified" error,

1. Right click GPView project (just under *Solution 'GPView'*)
2. Click Properties
3. Under Configuration Properties -> Debugging, find Command Arguments
4. GPView takes the model file as a command argument, so write the path to the model file. You can use relative paths (e.g. ../Models/Hammer.SAT)
