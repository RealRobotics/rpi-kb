#!/bin/bash
# Installs packages needed to connect to Eduroam.

sudo apt update
sudo apt install -y network-manager network-manager-gnome openvpn openvpn-systemd-resolved network-manager-openvpn network-manager-openvpn-gnome
sudo apt purge -y openresolv dhcpcd5
sudo ln -sf /lib/systemd/resolv.conf /etc/resolv.conf
