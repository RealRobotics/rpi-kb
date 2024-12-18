# Backup and restore an SD card

This page was based on this post: <https://www.guivi.one/2021/06/07/how-to-clone-raspberry-pi-sd-card-on-linux-and-shrink-it-to-actual-size/>

The basic process is:

1. Copy the full image onto your PC hard drive.
2. Shrink the image and add the necessary magic to uncompress the image on first boot using the [PiShrink utility](https://github.com/Drewsif/PiShrink).
3. Copy the shrunk image to the new SD card.

The minimal commands are detailed below.

## Install PiShrink

This is necessary as `pishrink` is run as root.

```bash
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
chmod +x pishrink.sh
sudo mv pishrink.sh /usr/local/bin
```

## Backup

Run the script [commands.bash](commands.bash).  The image is created and
shrunk in place and resultant file can be found here  `./myimg.img.gz`.

Move the file to a safe place.  Note `sudo` is needed to move the file as
it was created using `sudo`.  Change the name to something more meaningful
e.g. `mybot_20230120.img.gz`.

## Restore

Use the Raspberry Pi imager tool and select the image file and write it to
a new SD card.

Insert SD card into RPi and boot.  After a bit, it should all work.
