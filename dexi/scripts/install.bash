#!/usr/bin/env bash

echo Setting up dexi.service...

sudo cp "$(dirname "$0")/dexi.service" /etc/systemd/system/
sudo cp "$(dirname "$0")/dexi_avr.service" /etc/systemd/system/
sudo cp "$(dirname "$0")/pigpiod.service" /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable dexi.service
sudo systemctl restart dexi.service
#sudo systemctl enable pigpiod.service
#sudo systemctl restart pigpiod.service

#sudo systemctl disable dexi.service
#sudo systemctl enable dexi_avr.service