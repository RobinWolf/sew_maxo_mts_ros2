#!/bin/bash

# Set static IP configuration using NetworkManager
nmcli connection modify 'netplan-eth0' ipv4.addresses 192.168.10.222/24
nmcli connection modify 'netplan-eth0' ipv4.method manual
necli connection modify 'netplan-eth0' connection.autoconnect yes
nmcli connection modify 'netplan-eth0' connection.autoconnect-retries -1
nmcli connection up 'netplan-eth0'