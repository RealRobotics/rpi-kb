#!/bin/bash

set -e

INTERNET_IF="wlp0s20f3"
ROBOT_NETWORK_IF="enp0s31f6"

echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
sudo iptables -t nat -A POSTROUTING -o ${ROBOT_NETWORK_IF} -j MASQUERADE
sudo iptables -A FORWARD -i ${INTERNET_IF} -o ${ROBOT_NETWORK_IF} -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i ${ROBOT_NETWORK_IF} -o ${INTERNET_IF} -j ACCEPT
# sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
sudo systemctl restart systemd-networkd

sudo iptables -L
sudo iptables -L -t nat
