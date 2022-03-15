# Aarhus University Robotics Platform (AURP) Overview

Here goes the readme...


## USB driver permissions

```
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d[0-9][0-9]", MODE="0666"' | sudo tee /etc/udev/rules.d/50-odrive.rules
sudo udevadm trigger
```

Then reconnect the odrive usb cable

