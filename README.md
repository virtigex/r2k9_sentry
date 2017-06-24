# R2K9 Manual

## Requirements

* Intel Euclid
* Kobuki Robot base

## Installation

* Change passwords for login, Wi-Fi and x11nvc.
* sudo apt install openssh-server git screen python-pip exfat-fuse exfat-utils speech-dispatcher
* pip install --upgrade pip
* pip install tensorflow

## Operation

Start Kobuki Base in a terminal

```
kobuki
```

Start R2K9 sentry

```
rosrun r2k9 r2k9_sentry
```
Control-C (SIGINT) to exit.

To enable data collection

```
rosparam set /r2k9/record /your/training/directory
```
To turn off data collection

```
rosparam delete /r2k9/record
```

## Speech

R2K9 can be programmed to speech via `spd-say`, if installed by setting parameters 
```
rosparam set /r2k9/say/startup "Hello, I am R2K9"
rosparam set /r2k9/say/shutdown "R2K9 is going to sleep now"
rosparam set /r2k9/say/track_start "Hi there"
rosparam set /r2k9/say/track_stop "Goodbye!"
```

## TODO

* Write launch files
* Respond to base going on and off line

