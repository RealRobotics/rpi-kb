l# How to connect to eduroam

This is pretty straightforward once you know how.

Mostly based off this:
<https://scholarslab.lib.virginia.edu/blog/raspberry-pi-uva-eduroam/>
but with tweaks from a working setup.

## Install NetworkManager

You need to install network manager first.  To do this, connect to your phone hotspot and once connected run the following commands:

```bash
sudo apt update
sudo apt install network-manager network-manager-gnome
```

Then modify the following files:

sudo nano /etc/dhcpcd.conf

Add this line to the bottom of the file, then save and close.

denyinterfaces wlan0

Then do this:

sudo nano /etc/NetworkManager/NetworkManger.conf

Add dchp=internal to the file and change managed to true, then save and close the file.

Reboot.

Eduroam can now be selected on the Network Manager icon.

# cat /media/andy/root/etc/NetworkManager/system-connections/eduroam 1
[wifi-security]
key-mgmt=wpa-eap

[connection]
id=eduroam
uuid=c403791d-481d-4864-837c-28d967421f50
type=wifi
timestamp=1581693570

[ipv6]
method=auto
ip6-privacy=0

[wifi]
ssid=eduroam
mode=infrastructure
mac-address=B8:27:EB:15:D5:88
seen-bssids=00:2C:C8:9A:8B:41;00:2C:C8:9D:1A:A1;
security=802-11-wireless-security

[802-1x]
eap=peap;
identity=men6rjc@leeds.ac.uk
phase2-auth=mschapv2
THIS IS THE IMPORTANT BIT
CHANGE
password=XXXXXXXXXX
TO
password-flags=2

[ipv4]
method=auto
cat: 1: No such file or directory


## Set locale

I like my keyboards to work correctly, so we we need to set up the locale correctly as Raspbian defaults to `en_US`.  Use `sudo raspi-config` to set the locale to `en_GB.utf8`, the time zone to `Europe/London` and the keyboard to 105 key, UK.




