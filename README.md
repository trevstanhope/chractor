# Chractor
Embedded system for Cherokee XJ traction, suspension, and engine monitoring

## Installation
Install Debian Linux 8.3.0 (headless) to the device.

Afterwards, copy-paste the following in the device's terminal:

    su
    cd /root
    git clone https://github.com/trevstanhope/chractor
    cd chractor
    sh install.sh

## Parts
All components required for the device are listed below

### Regulator
16VDC~40VDC to 12VDC 10A 120W Waterproof Voltage Convert Transformer

* https://www.amazon.com/gp/product/B01ARRA69A/ref=ox_sc_act_title_1?ie=UTF8&psc=1&smid=A1THAZDOWP300U

### Sensors
DS18B20 temperature sensor

* https://www.amazon.com/gp/product/B00CHEZ250/ref=ox_sc_act_title_2?ie=UTF8&psc=1&smid=A68PK47R45UCB

Dorman #924-261 suspension sensor

* http://www.ebay.com/itm/371313000320?_trksid=p2060353.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT

Adafruit 10-DOF IMU (LSM303DLHC, L3GD20, and BMP180)

* https://www.adafruit.com/product/1604
* https://github.com/adafruit/Adafruit_10DOF

### Enclosure
Cinch ModICE LE

* http://www.peigenesis.com/en/about-us/5540:partner-with-distributors-for-custom-connector-designs.html

### Connectors
Bulgin 400 Series 

* https://www.digikey.com/product-detail/en/bulgin/PX0413-08S-PC/708-1083-ND/1625848
* https://www.digikey.com/product-detail/en/bulgin/PX0410-08P-6065/708-1034-ND/1625799
* https://www.digikey.com/product-detail/en/bulgin/PX0441-4M50/708-1231-ND/1625996
* https://www.digikey.com/product-detail/en/bulgin/PX0443/708-1235-ND/1626000
