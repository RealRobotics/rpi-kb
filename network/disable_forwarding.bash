#!/bin/bash

set -e

INTERNET_IF="wlp0s20f3"
ROBOT_NETWORK_IF="enp0s31f6"

echo 0 | sudo tee /proc/sys/net/ipv4/ip_forward
sudo iptables -t nat -D POSTROUTING -o ${INTERNET_IF} -j MASQUERADE
sudo iptables -D FORWARD -i ${INTERNET_IF} -o ${ROBOT_NETWORK_IF} -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -D FORWARD -i ${ROBOT_NETWORK_IF} -o ${INTERNET_IF} -j ACCEPT
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
sudo service networking restart

sudo iptables -L -t nat
