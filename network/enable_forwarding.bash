#!/bin/bash

set -e

INTERNET_IF="wlp0s20f3"
ROBOT_NETWORK_IF="enp0s31f6"

echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
sudo iptables -A FORWARD -i ${ROBOT_NETWORK_IF} -o ${INTERNET_IF} -j ACCEPT
sudo iptables -A FORWARD -i ${INTERNET_IF} -o ${ROBOT_NETWORK_IF} -m state --state ESTABLISHED,RELATED -j ACCEPT
sudo iptables -t nat -A POSTROUTING -o ${INTERNET_IF} -j MASQUERADE

sudo iptables -L -t nat
