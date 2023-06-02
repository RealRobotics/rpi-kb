#!/bin/bash

set -e

INTERNET_IF="wlp0s20f3"
ROBOT_NETWORK_IF="enp0s31f6"
ROBOT_IP_RANGE="192.168.53.2/24"

# echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
# sudo iptables -t nat -A POSTROUTING -o ${ROBOT_NETWORK_IF} -j MASQUERADE
# sudo iptables -A FORWARD -i ${INTERNET_IF} -o ${ROBOT_NETWORK_IF} -m state --state RELATED,ESTABLISHED -j ACCEPT
# sudo iptables -A FORWARD -i ${ROBOT_NETWORK_IF} -o ${INTERNET_IF} -j ACCEPT
# sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
# sudo systemctl restart systemd-networkd

# https://serverfault.com/questions/959288/iptables-bridge-routing-between-two-independent-lans-and-router

# This doesn't work either...
# sudo iptables -A FORWARD -i wlp0s20f3 -j ACCEPT
# sudo iptables -t nat -A POSTROUTING -o enp0s31f6 -j MASQUERADE

echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward

# Can ping with this but DNS not working.

iptables -A FORWARD -i enp0s31f6 -o wlp0s20f3 -j ACCEPT
iptables -A FORWARD -i wlp0s20f3 -o enp0s31f6 -m state --state RELATED,ESTABLISHED -j ACCEPT

# This allows pining of 8.8.8.8 without breaking the PC.  No DNS yet...
iptables -t nat -A POSTROUTING -s 192.168.53.0/24 -o wlp0s20f3 -j MASQUERADE

# DNS  - None of these work but they don't break anything.
# iptables -A INPUT -p udp --dport 53 -j ACCEPT
# iptables -A OUTPUT -p udp --sport 53 -j ACCEPT
# iptables -A INPUT -p tcp --dport 53 -j ACCEPT
# iptables -A OUTPUT -p tcp --sport 53 -j ACCEPT

# This is nicer as it only controls the subnet but it still does nothing!
iptables -A INPUT -s 192.168.53.0/24 -p udp --dport 53 -j ACCEPT
iptables -A INPUT -s 192.168.53.0/24 -p tcp --dport 53 -j ACCEPT
