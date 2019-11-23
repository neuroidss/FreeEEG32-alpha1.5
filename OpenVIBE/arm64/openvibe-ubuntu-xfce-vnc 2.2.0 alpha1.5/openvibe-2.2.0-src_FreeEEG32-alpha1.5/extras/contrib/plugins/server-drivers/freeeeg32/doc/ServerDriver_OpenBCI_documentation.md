![OpenViBE logo][OpenViBELogo]

![OpenBCI logo][OpenBCILogo]

# OpenBCI driver for OpenViBE documentation #

(documentation stands for [OpenViBE][OpenViBE] **1.2.0-OpenBCI-RC1**)

## Welcome ##

Welcome in the documentation of the [OpenBCI][OpenBCI] driver for [OpenViBE][OpenViBE]. This document gives you step by step details on how to configure your system, hardware and software to get the best possible experience. For further details about OpenBCI and OpenViBE, please refer to their [respective][OpenBCI] [documentation][OpenViBE].

## Getting Started ##

This section gives guidelines on how to set-up your environment configuration to get the best experience using [OpenBCI][OpenBCI] with [OpenViBE][OpenViBE]

### Linux ###

In order to use the [OpenBCI][OpenBCI] driver on Linux, you need to be granted read/write access to the serial port that is used to communicate with the dongle. Unfortunately, this does not come as a standard configuration on many Linux distributions and you will need to configure your setup so that the user is granted these access. On [Ubuntu][UbuntuDotCom] and its derivative, as well as on [Debian][DebianDotOrg], this can simply be achieved by adding the user to the `dialog` group with the following command :

> sudo adduser *username* dialout

Please note that you may need to close and reopen the session for changes to take effect.

On [Fedora][FedoraDotOrg] and its derivative, a similar command should be used to add the user in the `dialout` group :

> sudo usermode -a -G dialout *username*

Again, please note that you may need to close and reopen the session for changes to take effect.

### Windows ###

It is necessary to walk through a litlle bit of configuration to have the best experience with [OpenBCI][OpenBCI] and [OpenViBE][OpenViBE] on Windows. Indeed, when Windows first installs a serial port, its default configuration is not optimal for realtime streaming of the data. It especially configures large buffering and latencies, causing delays in the communication which eventually lead in delays in the acquired signals. While this only delays the detection of Motor Imagery or SSVEP tasks by a few hundreds of milliseconds, this also delays the ERP (eventually leading to a misdetection) and in all cases, alters the neurophysiological validity of the acquired data by a significant fraction. Consequently, it is recommended to configure the serial ports so that they are suitable for realtime use.

Note that this configuration is necessary each time Windows creates a new serial port. This may unfortunately happen **every time** you move the dongle on a new USB port on the computer !

In order to open the configuration dialog of the serial port, please follow these steps:

- Right click on the `Computer` icon and chose `Manage`
- After the `Computer Management` tool is opened, browse the tree in the left pane to the `Device Manager` section
- After the `Device Manager` tree is opened, browse in the right pane to the `Ports (COM & LPT)` section
- Then double click your port (in my case, `USB Serial Port (COM3)`

![The Windows Device Manager][WindowsTweaking1]

- In the `USB Serial Port Properties` dialog, go to the `Port Settings` tab and click `Advanced`

![The Port Settings dialog][WindowsTweaking2]

- This opens the `Advanced Settings for your COM port`

![The Advanced Port Settings dialog][WindowsTweaking3]

From this window, there are three settings we need to change :

- The `Receive` and `Transmit` settings of the `USB Transfer Sizes` section should both be changed from 4096 to **64** as this will reduce the time taken by the OS to deliver the data to either the dongle or the [OpenViBE][OpenViBE] driver
- The `Latency Timer` of the `BM Options` section should be reduced down to **1** as this will dramatically reduce the latency induced by the OS.

After you changed the settings as shown on the following figure, please click `OK`.

![The tweaked configuration of COM ports to have best experienece with OpenViBE and OpenBCI][WindowsTweaking4]

It is *not necessary* to restart the computer for the changes to take effect.

## Configuration ##

The configuration dialog of the [OpenBCI][OpenBCI] driver comes with a number of settings that one can change or pick depending on his preference.

![The OpenBCI configuration dialog][DriverDialog]

The following table documents each option that is specific to the [OpenBCI][OpenBCI] device. Please refer to the [OpenViBE documentation][OpenViBEDoc] for anything generic to the [OpenViBE][OpenViBE] Acquisition Server.

| Option | Default Value | Documentation |
| :-------------------------: | :-------------------------: | :-----------------------------------------------------------------------------------|
| **Device** | *empty* | This allows you to pick a serial port to connect on. The drodown list shows the serial ports that can currently be opened on this computer. If no port is found, the mention *No valid serial port* is shown in this list. If you cannot find your device in this list, please refer  . |
| **Use Daisy Module** | *false* | This allows you to configure the daisy module. Four cases should be considered. 1/ if the daisy module is present and this option is set to **true**, then the device will turn to 16 channels samples 125 Hz. 2/ if no daisy module is present and this option is set to **false**, then the device will turn to 8 channels, 250 Hz. 3/ if the daisy module is **not present** on the board and this option is set to **true**, then the initialization of the driver will **fail**. 4/ if the daisy module is **present** on the board and this option is set to **false**, the daisy module will be disabled and the acquisition will be done as if the daisy module was not present on the board, turning the device back to 8 channels sampled at 125 Hz. |
| **Custom Command On Initialization** | *empty* | This option contains additional commands to send to the device at initialization. You must use one line per command, some command may contain multiple characters. For details about the commands, please refer to the [OpenBCI protocol documentation][OpenBCIProto]. Be advised that this will increase the delay of initialization by an order of magnitude that is a direct relation of the number and types of commands you want to add. Finally, not all the commands take the same time to be executed, if you include custom commands, you should consider adjusting the timeout values. |
| **Board Reply Reading Timeout** | 5000 | This allows to define the maximum time until reading a reply from the board after sending a command times out. Many commands end with a **\$\$\$** pattern, which can handily be captured and release the waiting loop when reading the board reply, but not all the commands have this **\$\$\$** pattern. Consequently, it is necessary to have a timeout for the other commands. The default value has been chosen to behave well even with custom commands that need a long time to reply such as **?**. If you don't use such command in your *Custom Command On Initialization*, you may reduce that delay. But be aware that if you reduce it too much, the driver may miss the **\$\$\$** pattern even though the board has sent it, resulting in unexpected behavior. |
| **Board Reply Flushing Timeout** | 500 | This option allows to flush and get rid of the streaming buffer. This is especially used when the driver asks the board to stop streaming and makes the streaming state absolutely clean when the driver needs to send a new command after stopping the streaming. You may reduce this value to make (re)connection faster, but if the buffer came not to be completely flushed, the remaining would be taken as the begining of the next command and this may result in unexpected behavior. |

The Configuration Summary gives information on the current configuration, especially the number of channels and the sampling rate of the device.

## Advanced Configuration ##

In addition to the above settings, another few settings are available to the user as advanced configuration. They are not exposed in the GUI and should be directly set in the [OpenViBEConfig][OpenViBE configuration] file instead. Refere to the [Configuration Manager section][OpenViBEConfig] of the [OpenViBE documentation][OpenViBEDoc] for further details on the configuration file format, location and others.

| Token | Default Value | Documentation |
| :-------------------------: | :-------------------------: | :-----------------------------------------------------------------------------------|
| **AcquisitionDriver OpenBCI MissingSampleDelayBeforeReset** | *1000* | This defines the size of the window to continuously monitor reception of samples from the driver. If no sample is received within that timeframe, the board is requested to stop and restart streaming. While the non-reception of samples from the board may reflect an unexpected state in the board, this strategy seems to sometimes recover and let the streaming go back to normal. The default value allows a good compromise between dealing with buffering and actual transmission delays and recovering fast when something goes wrong. If you experience such unstability in the transmision, we recommend that you first explore anything that may (in)directly affect the quality of the transmission before tweaking this setting. |
| **AcquisitionDriver OpenBCI DroppedSampleCountBeforeReset** | *5* | This defines the number of sample loss events until a recovery is attempted. It happens that the board gets in an unstable state where some sample would be missing in the stream. Stopping and restarting the streaming has proved to recover well. The default setting has been set so that a few occasional sample loss may occur (due to e.g. quality transmission) and be corrected by the drift correction process, while not waiting too long to attempt recovery when too many sample are lost. |
| **AcquisitionDriver OpenBCI DroppedSampleSafetyDelayBeforeReset** | *1000* | This defines a sefety delay where no reset should be attempted because of sample loss (see **AcquisitionDriver OpenBCI DroppedSampleCountBeforeReset**). This prevents a reset on the first sample where the driver synchronises with the streaming protocol and may miss a few samples until it is perfectly synced with the header and tail of the protocol frame. |

[FedoraDotOrg]: http://www.fedora.org
[UbuntuDotCom]: http://www.ubuntu.com
[DebianDotOrg]: http://www.debian.org

[OpenViBE]: http://openvibe.inria.fr
[OpenViBEDoc]: http://openvibe.inria.fr/documentation-index
[OpenViBEConfig]: http://openvibe.inria.fr/the-configuration-manager
[OpenViBELogo]: http://openvibe.inria.fr/openvibe/wp-content/themes/openvibe/images/openvibe-banner.png

[OpenBCI]: http://docs.openbci.com
[OpenBCIProto]: http://docs.openbci.com/software/01-OpenBCI_SDK
[OpenBCILogo]: http://openbci.com/community/wp-content/uploads/2015/09/logo_wide_web_borders_BIG-copy-1024x137.png

[WindowsTweaking1]: ServerDriver_OpenBCI_windows_tweaking_1.png
[WindowsTweaking2]: ServerDriver_OpenBCI_windows_tweaking_2.png
[WindowsTweaking3]: ServerDriver_OpenBCI_windows_tweaking_3.png
[WindowsTweaking4]: ServerDriver_OpenBCI_windows_tweaking_4.png
[DriverDialog]: ServerDriver_OpenBCI_configuration.png

