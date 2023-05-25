# Networking for robots

The use case is as follows:

- The base station PC (normally someone's laptop) is connected to the internet via Wi-Fi.
- The PC is connected to a Wi-Fi router via an Ethernet cable.
- The Wi-Fi router is used to connect to the robots (often Raspberry Pi based).

## Set up router

The first thing to do is to do the basic setup of the Wi-Fi router.  The details of this may vary from router to router but the principles should be the same.

1. Connect to the router using your PC following the instructions in the router manual. I did the following:
   1. Factory Reset the router.
   2. Connect the router to the PC and log in to the web based UI.
2. Set the LAN IP address to `192.168.xx.1` (where `xx` is a number of your choice) and set the subnet to `255.255.255.0`.  You will probably need to reboot for this change to take effect.
3. Change the wireless SSID name and password to something more memorable for the 2.4GHz and 5GHz wireless adapters.  Save changes.

## Configure the PC

TODO

## Notes

These are the failed attempts.

### Share same subnet without internet



### Simple internet sharing

I first tried to follow the simplest thing that I thought would work.
<https://askubuntu.com/questions/1104506/share-wireless-internet-connection-through-ethernet-on-18-04-lts> looked like it should work.

To do this, plug in the Ethernet cable to you PC and the other end into the LAN side of the router (yellow sockets).  The IP address for Ethernet post on the CP was set to `10.42.0.1`.  Connect another PC to the router and change the LAN IP address for the router to `10.42.0.2` and save, probably needing a reboot as well.

I was able to ping the router and the attached laptop from my PC and vice versa.  However, due to the subnet limiting, I was not able to access the internet.

When I tested it, the router connected to the connection from the PC and I was able to ping the