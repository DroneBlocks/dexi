FULL_MAC=$(cat /sys/class/net/wlan0/address)

# echo "FULL MAC = $FULL_MAC"

PARTIAL_MAC=$(echo $FULL_MAC | awk -F: '{print $(NF-1)$NF}')

# echo "PARTIAL MAC = $PARTIAL_MAC"

DEXI_SSID="dexi_$PARTIAL_MAC"

echo $DEXI_SSID
