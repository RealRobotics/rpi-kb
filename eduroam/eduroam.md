# How to connect to eduroam

Most of the instructions were found in [this IT knowledge base article, method 2.](https://it.leeds.ac.uk/it?id=kb_article&sysparm_article=KB0012058)  I have copied them here to save looking them up and added extra notes.

Tested on the latest Raspberry Pi OS (64 bit) based on the Debian Bullseye release
(4 April 2022).

## Install NetworkManager

1. You need to install network manager first.  To do this, connect the Raspberry Pi to your phone hotspot (or teher to your phone using Bluetooth) and once connected run the following commands:

    ```bash
    sudo apt update
    sudo apt install network-manager network-manager-gnome openvpn openvpn-systemd-resolved network-manager-openvpn network-manager-openvpn-gnome
    sudo apt purge openresolv dhcpcd5
    sudo ln -sf /lib/systemd/resolv.conf /etc/resolv.conf
    ```

    __DO NOT DO A FULL UPGRADE USING YOUR PHONE.__ The full update can be done once you are on Eduroam, saving lots of your precious data!
2. Reboot.
3. The network manager icon should now be shown at the top of the screen.  Select Eduroam and try to connect.  A dialog box should be shown where you can enter the following info.
    | | |
    |---|---|
    |Security: |`WPA/WPA2 Enterprise`|
    |Authentication:| `Protected EAP (PEAP)`|
    |PEAP version:| `Automatic`|
    |Inner authentication:| `MSCHAPV2`|
    |Username:| `username@leeds.ac.uk`|
    |Password:| `xxxxxxxx`|
    TODO : REtest this and take screen shots.  It think there is a check box to tick.

    All other fields should be left blank.  Replace `username` with your username and `xxxxxxxx` with your password.  Press `OK` and then try connecting to Eduroam.  If all goes well, you should be able to connect.
4. Finally for this section, remove the old network applet as follows.  Right click menu bar on top of screen -> open "Panel Settings" -> "Panel Applets": remove "Wireless & Wired Network".  To keep things tidy, you also need to remove the spacer.  Right click on the space where the network icon used to be and select 'Remove "Spacer"'.

## Forcing password entry

As the password that you have just entered is held in a plain text file, it is much more secure to force the user to enter their password every time they connect to the network.

1. Ensure that you are disconnected from Eduroam.
2. Edit the file `/etc/NetworkManager/system-connections/eduroam` using `sudo nano` or similar.
3. Find section `[802-1x]` and change the line `password=XXXXXXXXXX` to   `password-flags=2`.
4. Save and exit the editor.
5. Try to connect to Eduroam.  You should now be prompted to enter your password.  If you are not prompted and just have a dialog box "Connect" and "Cancel" button, use `nmtui` instead, to enter your username and password, see below for details.

## Changing the username

TODO Test that this is possible.

## Using `nmtui`

The `nmtui` utility is an ncurses implementation of the Network Manager application and because it can be invoked with `sudo`, it works when the full Network Manager application doesn't behave.

The following shows you how to enter your `eduroam` user name and password.

1. In a terminal window, enter `sudo nmtui`.
2. Use the arrow keys to select "Activate a connection" and press enter.
3. Select `eduroam` and press enter.
4. You should now be prompted to enter your username and password.  Do this and press enter.  After a few seconds, you should be connected.

### Example system connection file

[This is what a working system connection file looks like.](working_files/eduroam)
