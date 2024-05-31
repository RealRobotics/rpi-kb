# Buried Pipe Network Setup

This document explains how a Wi-Fi network was used to provide full Wi-Fi coverage to a buried pipe network to robots controlled by a Raspberry Pi 5.  The principles outlined here should be applicable to similar Wi-Fi devices. With the benefit of hindsight, a full mesh network solution would have been better, as usual, time was short so we had to do what we could with what we had.

The chosen Wi-Fi devices were 4 identical GL.iNet GL-SFT1200 Opal devices.  The Opal devices are small, portable and can be run off a USB power supply are run a nice front end on top of OpenWRT so the software can easily changed to full OpenWRT if needed.  The switch used was a TP-Link TL-SG108S and was just used to connect the various active devices.

## Configuration details

Most of the following was gleaned from OpenWRT forum posts.  There are many little details to consider when configuring this system.  This is an overview of the system implemented.

```text
  ----     ------     ------------
 | PC |---|Switch|---|Wi-Fi Router| BF4
  ----     ------     ------------
            | | |     --------
            | | -----|Wi-Fi AP| E99
            | |       --------
            | |       --------
            | -------|Wi-Fi AP|
            |         --------
            |         --------
            ---------|Wi-Fi AP|
                      --------
```

1. One of the Opal routers will be used as a router.  This router will run the DHCP for the system.
2. The other 3 Opal routers will be configured as wireless access points.
3. The PC will get an IP address using DHCP over Ethernet.  A profile was created to do this.
4. All Opal routers will have the same SSID and SSID password. This will allow the users of this network to see it as same network.
5. To avoid channel clashes, inevitable on the 2.4GHz channels with 4 devices, 2.4GHz Wi-Fi was disabled.  The 5GHz band has many more channels so channel clashes can be avoided.  All channels use bandwidth of 20MHz.  The channels are set up as follows:

    | Device ID | IP address | Channel |
    |---|---|---|
    | BF4 | 192.168.62.1 | 40 |
    | C90 | 192.168.62.2 | 44 |
    | DF8 | 192.168.62.3 | 48 |
    | E98 | 192.168.62.4 | 52 |

    NOTE: The WAN port address is the ID plus 1, e.g. E98 has WAN port ending E99.

## Setup all Opal devices

The following process applies to all Opal devices.

1. Connect the device to the network switch and the PC using one of the LAN ports.
2. Log in to the admin panel.
3. Internet tab.  No changes.
4. Wireless tab.
   1. Disable 2.4GHz and both guest networks.
5. Clients tab.  No changes.
6. VPN tab.  No changes.
7. Applications tab.  No changes.
8. Network tab.  Router only.
   1. LAN tab.  Change Router IP address to 192.168.62.1. Apply changes.
9. System tab.
   1. Time zone.  Sync with browser.
   2. Advanced settings.  Open new browser tab with the OpenWRT LuCI web interface.
      1. Login with router admin password.
      2. Go to Network->Wireless page.
         1. Edit the only enabled page `radio1: Master`
            1. Device Configuration section. Click on advanced settings.
               1. Change country code to `GB - United Kingdom`.  Press Save button at bottom of page.
               2. Set channel number as per above table and bandwidth to 20MHz.
            2. Interface configuration.
               1. General setup tab.
                  1. Set ESSID to the correct value.
               2. Wireless security.
                  1. Set Key to SSID password value.
                  2. Enable fast transition.  Leave all other options alone.
               3. Save and apply.
      3. Log out.

## Setup access points

Apply the above settings.  Then do the following extra steps:

1. Plug the router into the network switch.
2. Plug the WAN port of the device into the network switch.
3. Navigate to `NETWORK->Network Mode`.
   1. Select the `Access Point` option and press `Apply`.
4. Swap the PC IP address over to the subnet 62.
5. Unplug the LAN connection from the device.
6. Login to the router.
7. Allocate a fixed IP address for the device using the MAC address.
   1. Navigate to the `NETWORK->LAN` page.
   2. Add the WAN mac address and set the IP address as per the table above.
   3. Unplug both cables from the device.
   4. Wait for a minute or so.
   5. Plug in the WAN port and verify that the IP address has changed correctly.
