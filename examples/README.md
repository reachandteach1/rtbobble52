In this folder are examples that can be run on any platform that supports a Bluetooth LE (BLE) adapter and has either a Chrome browser or Python. The HTML file can be run locally. If you choose to host the html file on a server, you must use https:// for it to work. The html file(s) only work under Chrome at this time. The Python files have only been tested under Python 3.8. 

The following Python libraries are required:
pip3 install requests bs4
pip3 install adafruit-io
pip3 install selenium
pip3 install --upgrade adafruit-blinka-bleio adafruit-circuitpython-ble
