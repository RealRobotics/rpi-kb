# How to connect to eduroam

Tested on the latest Raspberry Pi OS based on the Debian Bullseye release
(4 April 2022).


Based off this:
<https://scholarslab.lib.virginia.edu/blog/raspberry-pi-uva-eduroam/>
but with tweaks from a working setup.

This also looks useful but has not been tested.
<https://it.leeds.ac.uk/it?id=kb_article&sysparm_article=KB0012058>


## Install NetworkManager

You need to install network manager first.  To do this, connect to your phone hotspot and once connected run the following commands:

```bash
sudo apt update
sudo apt install network-manager network-manager-gnome
```

__DO NOT DO A FULL UPGRADE USING YOUR PHONE.__ The full update can be done once you are on Eduroam, saving lots of your precious data!

Then modify the following files:

```bash
sudo nano /etc/dhcpcd.conf
```

Add this line to the bottom of the file, then save and close.

```bash
denyinterfaces wlan0
```

Then do this:

```bash
sudo nano /etc/NetworkManager/NetworkManger.conf
```

Add `dchp=internal` to the file and change managed to true, then save and close the file.

Reboot.

Eduroam can now be selected on the Network Manager icon.

Connect to Eduroam filling out the following fields of the Wi-Fi Security Tab:
| | |
|---|---|
|Security: |`WPA/WPA2 Enterprise`|
|Authentication:| `Protected EAP (PEAP)`|
|Domain:| `leeds.ac.uk`|
|PEAP version:| `Automatic`|
|Inner authentication:| `MSCHAPV2`|
|Username:| `username@leeds.ac.uk`|
|Password:| `xxxxxxxx`|

All other fields should be left blank.  Replace `username` with your username and `xxxxxxxx` with your password.  Press `OK` and then try connecting to Eduroam.  If all goes well, you should be able to connect.

## Forcing password entry

As the password that you have just entered is held in a plain text file, it is much better to force the user to enter their password every time they connect to the network.

1. Ensure that you are disconnected from Eduroam.
2. List all system connection files using `ls /etc/NetworkManager/system-connections/`.
3. There may be more than one systems connection file for Eduroam,
4. Edit the file `/etc/NetworkManager/system-connections/eduroam` using `nano` or similar.
5. Find section `[802-1x]` and change the line `password=XXXXXXXXXX` to   `password-flags=2`.
6. Save and exit the editor.
7. Try to connect to Eduroam.  You should now be prompted to enter your password.

### Example system connection file

[This is what a working system connection file looks like.](eduroam/working_files/eduroam)
