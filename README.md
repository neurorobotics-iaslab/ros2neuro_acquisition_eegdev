# ROSNeuro Acquisition EEGDEV Plugin

The **EEGDEv** plugin allows to acquire neurodata from different commercial amplifiers. The plugin strongly relies on [libeegdev](https://neuro.debian.net/pkgs/libeegdev-dev.html) and [libxdffileio](https://neuro.debian.net/pkgs/libxdffileio-dev.html), two open source libraries available in neurodebian repository. The latest version of **libeegdev** is provided by the **ROS-Neuro** installation as debian package while **libxdffileio** is automatically installed from NeuroDebian repository.

## Usage
Instructions on how to install xdffileio and eegdev are provided below. Here, we will have a look the usage of the **EEGDEV Plugin**.

To launch the rosneuro_acquisition with the EEGDEV plugin, first it is required to set the `plugin` parameter to `rosneuro::EGDDevice`, then to provide the desired `framerate`. Notice that these are the two mandatory parameters for every plugin of rosneuro_acquisition.

### Plugin-specific parameters:
~<name>/`devarg` (`std::string`) [**mandatory**]
	
  The device name according to eegdev library.

~<name>/`samplerate` (`int`)
  
  Samplerate (in Hz) to be set in the device (if allowed by the hardware). If it is not provided, the default sampling rate of the device will be used.
  
~<name>/`devext` (`std::string`)
  
  Additional extra arguments for the specific device. Please refer to libeegdev for more details.

#### Possible devarg identifiers:
  | devarg | Device |
  |:------:|:------:|
  | \*.gdf | Fullpath of GDF file for playback |
  | \*.bdf | Fullpath of BDF file for playback |
  | gtec | g.USBamp from g.Tec |
  | eego | AntNeuro eego device |
  | biosemi | Biosemi device |
  | q20 | Cognionics Quick-20 device |
  | wsdsi | Wearable sensing device |
  | gtecnet | g.TecNet language support |
  | bbt | Bit Brain device |
  | neurone | Neurone device |
  | barv | BrainProduct RDA support |
  | tia | Tobi interface A support |
  | neurosky | Neurosky device |


## Source compilation and installation of libeegdev and libxdffileio
If there is the necessity to compile and install libeegdev and libxdffileio from source, you should follow these guidelines.

You can download the source code of the libraries from:
- [neurorobotics-iaslab/xdffileio](https://github.com/neurorobotics-iaslab/xdffileio)
- [neurorobotics-iaslab/eegdev](https://github.com/neurorobotics-iaslab/eegdev)

### 1. Setup the system

To compile the libraries you need to install the following packages: `sudo apt-get install libtool gnulib flex bison`.
Furthermore, **libeegdev** provides support for a variety of devices and some of them has proprietary drivers. **The drivers must be installed on the system to enable the support for the specific device.**
Before proceeding with the installation, we suggest creating a local directory in which the libraries will be saved:
```bash
cd $HOME && mkdir Local
```
Then, modify the .profile by copying and pasting the following lines at the end of the file:
```bash
# LOCAL
if [ -d "$HOME/Local" ] ; then
    export PATH="$HOME/Local/bin:$PATH"
    export LD_LIBRARY_PATH="$HOME/Local/lib/:$LD_LIBRARY_PATH"
    export LIBRARY_PATH="$HOME/Local/lib/:$LIBRARY_PATH"
    export CPATH="$HOME/Local/include/:$CPATH"
    export PKG_CONFIG_PATH="$HOME/Local/lib/pkgconfig/:$PKG_CONFIG_PATH"
    export LIB=$LD_LIBRARY_PATH:$LIB
    export INCLUDE=$HOME/Local/include/:$INCLUDE
fi
```
Reboot the session in order to see the modifications of the environmental variables.

### 2. Compile and install libxdffileio
First, clone the xdffileio repository:
```bash
git clone git@github.com:neurorobotics-iaslab/xdffileio.git
```
Enter in the `xdffileio` directory and run:
```bash
./autogen.sh
```
This script setup the folder with libtool and gnulib.

At this point, run the configure:
```bash
./configure --prefix=$HOME/Local
```
Compile and install the library:
```bash
make && make install
```
### 3. Compile and install libeegdev
As before, first clone the eegdev repository:
```bash
git clone git@github.com:neurorobotics-iaslab/eegdev.git
```
Enter in the `eegdev` directory and run:
```bash
./autogen.sh
```
This script setup the folder with libtool and gnulib.

At this point, run the configure:
```bash
./configure --prefix=$HOME/Local
```
**Please note** that if you want to install additional plugins you have to add --with-PLUGIN option to the configure. For instance, if we own the ANTNeuro SDK library and we want to install the eegdev plugin for ANTNeuro, we need to run the configure with the following options:
```bash
./configure --prefix=$HOME/Local --with-eego
```
Please refer to ```./configure --help``` for a list of available plugins.

**At the end of the configure, a table will show the supported devices (according to the driver installed in your machine).**
Compile and install the library:
```bash
make && make install
```

