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
DS18B20 Temperature Sensor

* https://www.amazon.com/gp/product/B00CHEZ250/ref=ox_sc_act_title_2?ie=UTF8&psc=1&smid=A68PK47R45UCB

Dorman #924-261 Suspension Sensor

* http://www.ebay.com/itm/371313000320?_trksid=p2060353.m2749.l2649&ssPageName=STRK%3AMEBIDX%3AIT

Adafruit 10-DOF IMU (LSM303DLHC, L3GD20, and BMP180)

* https://www.adafruit.com/product/1604
* https://github.com/adafruit/Adafruit_10DOF

### Enclosure
Cinch ModICE LE

* http://www.peigenesis.com/en/about-us/5540:partner-with-distributors-for-custom-connector-designs.html

### Connectors
Sensor Harness, Bulgin 400 Series (8 position)

* Female Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0413-08S-PC/708-1083-ND/1625848
* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-08P-6065/708-1034-ND/1625799
* Male Pins (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3348/708-1089-ND/1625854
* Female Sockets (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3347-1/708-1093-ND/1625858

Power Supply, Bulgin 400 Series (2 position)

* Male Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0413-02P-PC/708-1072-ND/1625837
* Female Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-02S-5560/708-1037-ND/1625802
* Male Pins (20-24 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3350-1/708-1092-ND/1625857
* Female Sockets (20-24 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3349-1/708-1091-ND/1625856

Camera Adaptor, Bulgin 400 Series (4 position)

* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-04P-4550/708-1032-ND/1625797
* Female Receptacle (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0411-04S-4550/708-1053-ND/1625818
* Male Pins (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3348/708-1089-ND/1625854
* Female Sockets (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3347-1/708-1093-ND/1625858

Camera Input, Bulgin Data Series (USB Mini)

* Extension Cable - https://www.digikey.com/product-detail/en/bulgin/PX0441-4M50/708-1231-ND/1625996
* Female Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0443/708-1235-ND/1626000

Switched Power Out, TE Connectivity (4 position)

* Female Receptacle (Panel-Mount) https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/206430-1/A1360-ND/19367
* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/206429-1/A1357-ND/19358
* Male Pins (20-24 AWG)- https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/2-66102-5/A31989TR-ND/808381

Firewall Port, Amphenol Industrial Operations (18.0mm ~ 25.0mm, M32x1.5)

* Strain Relief - http://www.digikey.com/scripts/DkSearch/dksus.dll?Detail&itemSeq=214306199&uq=636173608808318739
