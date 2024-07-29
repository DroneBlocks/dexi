#!/bin/bash

#make sure were sudo
sudo true

#determine the SSID to configure
FULL_MAC=$(cat /sys/class/net/wlan0/address)
PARTIAL_MAC=$(echo $FULL_MAC | awk -F: '{print $(NF-1)$NF}')
DEXI_SSID="dexi_$PARTIAL_MAC"

#capture the SSID and password

# found this gem to potentially help make this interactive in the future
# https://askubuntu.com/questions/75625/how-do-i-scan-for-wireless-access-points
# sudo iw wlan0 scan | egrep "SSID:" | sed -e "s/\tSSID: //" | awk '{ORS = (NR % 2 == 0)? "\n" : " "; print}' | sort -gr

wifi_pass=$(wpa_passphrase $1 $2 | grep psk= | grep -v '#psk=' | sed 's/[ \t]psk=//')

sudo ./raspiApWlanScripts/setup_wlan_and_AP_modes.sh -s $1 -p $wifi_pass  -a $DEXI_SSID -r droneblocks
