# Networking for robots

The use case is as follows:

- The base station PC (normally someone's laptop) is connected to the internet via Wi-Fi.
- The PC is connected to a Wi-Fi router via an Ethernet cable.
- The Wi-Fi router is used to connect to the robots (often Raspberry Pi based).

## Set up router

The first thing to do is to do the basic setup of the Wi-Fi router.  The details of this may vary from router to router but the principles should be the same.

1. Connect to the router using your PC following the instructions in the router manual. I did the following:
   1. Factory Reset the router.
   2. Connect the router to the PC and log in to the web based UI.
2. Set the LAN IP address to `192.168.xx.1` (where `xx` is a number of your choice) and set the subnet to `255.255.255.0`.  You will probably need to reboot for this change to take effect.
3. Change the wireless SSID name and password to something more memorable for the 2.4GHz and 5GHz wireless adapters.  Save changes.

## Configure the PC

TODO

### Share

Set up bridge.

IP tables.

 sudo sh -c "iptables -I INPUT -p tcp -m tcp --dport 80 -j ACCEPT && iptables -I INPUT -p tcp -m tcp --dport 443 -j ACCEPT && service iptables save"

This is easy.


### IP tables before changing anything

```text
$ sudo iptables -L -n
Chain INPUT (policy ACCEPT)
target     prot opt source               destination

Chain FORWARD (policy DROP)
target     prot opt source               destination
DOCKER-USER  all  --  0.0.0.0/0            0.0.0.0/0
DOCKER-ISOLATION-STAGE-1  all  --  0.0.0.0/0            0.0.0.0/0
ACCEPT     all  --  0.0.0.0/0            0.0.0.0/0            ctstate RELATED,ESTABLISHED
DOCKER     all  --  0.0.0.0/0            0.0.0.0/0
ACCEPT     all  --  0.0.0.0/0            0.0.0.0/0
ACCEPT     all  --  0.0.0.0/0            0.0.0.0/0

Chain OUTPUT (policy ACCEPT)
target     prot opt source               destination

Chain DOCKER (1 references)
target     prot opt source               destination

Chain DOCKER-ISOLATION-STAGE-1 (1 references)
target     prot opt source               destination
DOCKER-ISOLATION-STAGE-2  all  --  0.0.0.0/0            0.0.0.0/0
RETURN     all  --  0.0.0.0/0            0.0.0.0/0

Chain DOCKER-ISOLATION-STAGE-2 (1 references)
target     prot opt source               destination
DROP       all  --  0.0.0.0/0            0.0.0.0/0
RETURN     all  --  0.0.0.0/0            0.0.0.0/0

Chain DOCKER-USER (1 references)
target     prot opt source               destination
RETURN     all  --  0.0.0.0/0            0.0.0.0/0
```

### Ideas

https://serverfault.com/questions/563441/bridge-vlan-and-internet-access-how

Router - commands?

eth0 (192.168.1.0/24) <==> eth1 (Public Internet)

NAT

iptables -I POSTROUTING -t nat -o eth1 -j MASQUERADE

https://serverfault.com/questions/959288/iptables-bridge-routing-between-two-independent-lans-and-router

iptables -A PREROUTING -t nat -p tcp -i eth0 --dport 8080 -j DNAT --to 192.168.2.1:80

iptables -A POSTROUTING -t nat -d 192.168.55.0/24 -o eth0 -s 192.168.2.0/24 -j MASQUERADE

iptables -A POSTROUTING -t nat -p tcp -d 192.168.55.0/24 -o eth0 -s 192.168.2.1 --sport 80 -j SNAT --to-source 192.168.55.2:8080


https://bwachter.lart.info/linux/bridges.html

```text
# allow ssh, smtp and http on the router _itself_ (INPUT!)
iptables -A INPUT -p tcp --dport 22 -m physdev --physdev-in eth1 -j ACCEPT
iptables -A INPUT -p tcp --dport 25 -m physdev --physdev-in eth1 -j ACCEPT
iptables -A INPUT -p tcp --dport 80 -m physdev --physdev-in eth1 -j ACCEPT

# reject all other connections to the router
iptables -A INPUT -p tcp --syn -m physdev --physdev-in eth1 -J REJECT

# allow the some on the FORWARD chain
iptables -A FORWARD -p tcp --dport 22 -m physdev --physdev-in eth1 --physdev-out eth0 -j ACCEPT
iptables -A FORWARD -p tcp --dport 25 -m physdev --physdev-in eth1 --physdev-out eth0 -j ACCEPT
iptables -A FORWARD -p tcp --dport 80 -m physdev --physdev-in eth1 --physdev-out eth0 -j ACCEPT
```

https://askubuntu.com/questions/875395/iptables-and-bridging

```text
iptables -F
iptables -F -t mangle
iptables -F -t nat

iptables -F INPUT
iptables -F FORWARD
iptables -F OUTPUT

iptables -X

iptables -P INPUT DROP
iptables -P FORWARD DROP
iptables -P OUTPUT DROP

iptables -A INPUT -m state --state ESTABLISHED,RELATED -j ACCEPT

#SSH
iptables -A INPUT -i br0 -p tcp --dport 22 -j ACCEPT

#HTTP(S)
iptables -A INPUT -p tcp -m multiport --dport 80,443 -j ACCEPT

#DNS
iptables -A INPUT -p tcp --dport 53 -j ACCEPT
iptables -A INPUT -p udp --dport 53 -j ACCEPT

#DNS
iptables -A INPUT -p tcp --dport 953 -j ACCEPT

#DHCP
iptables -A INPUT -p udp --dport 67 -j ACCEPT
iptables -A INPUT -p udp --dport 68 -j ACCEPT

iptables -A OUTPUT -o lo -j ACCEPT
iptables -A OUTPUT -o br0 -j ACCEPT

#DNS
iptables -A OUTPUT -p tcp -m tcp --sport 53:65535 --dport 53 -j ACCEPT
iptables -A OUTPUT -p udp -m udp --sport 53:65535 --dport 53 -j ACCEPT

iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE

#HTTP(S)
iptables -A FORWARD -p tcp -m multiport --dport 80,443 -j ACCEPT
iptables -A FORWARD -p udp -m multiport --dport 80,443 -j ACCEPT

#DNS
iptables -A FORWARD -p tcp --dport 53 -j ACCEPT
iptables -A FORWARD -p udp --dport 53 -j ACCEPT

#NTP
iptables -A FORWARD -p udp --dport 123 -j ACCEPT

iptables -A FORWARD -i wlan0 -o eth1 -j ACCEPT
iptables -A FORWARD -i wlan0 -o eth1 -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -A FORWARD -i eth1 -o wlan0 -j ACCEPT
iptables -A FORWARD -i eth1 -o wlan0 -m state --state ESTABLISHED,RELATED -j ACCEPT

iptables -A FORWARD -m state --state RELATED,ESTABLISHED -j ACCEPT

iptables -A FORWARD -s 192.168.2.0/24 -d 192.168.2.0/24 -j ACCEPT
```

https://serverfault.com/questions/431593/iptables-forwarding-between-two-interface

wlan0 (station) - Connected to the internet connection

wlan1 (AP) - Other clients connect to it.

```text
echo 1 > /proc/sys/net/ipv4/ip_forward
iptables -A FORWARD -i wlan1 -o wlan0 -j ACCEPT
iptables -A FORWARD -i wlan0 -o wlan1 -m state --state ESTABLISHED,RELATED -j ACCEPT
iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
```

This looks nice and easy.

## Notes

These are the failed attempts.

### Share same subnet without internet

This is easy.

### Share same subnet without internet using a bridge

The following commands created a bridge that allowed me to ping both ways.

```bash
sudo ip link add name uol_robotics_br type bridge
sudo ip link set dev uol_robotics_br up
sudo ip link set enp0s31f6 master uol_robotics_br
bridge link
sudo ip address add 192.168.53.2/24 dev uol_robotics_br
```

Verify that the route is set up correctly.

```bash
$ ip route
default via 192.168.2.1 dev wlp0s20f3 proto dhcp metric 600
169.254.0.0/16 dev wlp0s20f3 scope link metric 1000
172.17.0.0/16 dev docker0 proto kernel scope link src 172.17.0.1 linkdown
192.168.2.0/24 dev wlp0s20f3 proto kernel scope link src 192.168.2.26 metric 600
192.168.53.0/24 dev uol_robotics_br proto kernel scope link src 192.168.53.2
```

Still can't access the internet from the PC on the subnet.

### Simple internet sharing

I first tried to follow the simplest thing that I thought would work.
<https://askubuntu.com/questions/1104506/share-wireless-internet-connection-through-ethernet-on-18-04-lts> looked like it should work.

To do this, plug in the Ethernet cable to you PC and the other end into the LAN side of the router (yellow sockets).  The IP address for Ethernet post on the CP was set to `10.42.0.1`.  Connect another PC to the router and change the LAN IP address for the router to `10.42.0.2` and save, probably needing a reboot as well.

I was able to ping the router and the attached laptop from my PC and vice versa.  However, due to the subnet limiting, I was not able to access the internet.

When I tested it, the router connected to the connection from the PC and I was able to ping the