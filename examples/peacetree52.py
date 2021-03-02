# Peacetree display using a rtbobble52
import requests
import time
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

ble = BLERadio()
while True:
    while ble.connected and any(
        UARTService in connection for connection in ble.connections
    ):
        for connection in ble.connections:
            if UARTService not in connection:
                continue
            print("RTBobble52 ready...")
            uart = connection[UARTService]
            while connection.connected:
                #fetch peacetree data from io.adafruit.com
                pt = requests.get('https://io.adafruit.com/api/v2/reachandteach/feeds/peacetree/')
                pt_json=pt.json()

                #convert the data into a bobble52 command
                bobble_command = pt_json['last_value']+' n'
                print(bobble_command)

                #output the data to the bobble52
                uart.write(bobble_command.encode("utf-8"))
                uart.write(b'\n')
                for i in range(12):
                    time.sleep(5)

    print("disconnected, scanning")
    for advertisement in ble.start_scan(ProvideServicesAdvertisement, timeout=1):
        if UARTService not in advertisement.services:
            continue
        ble.connect(advertisement)
        print("connected")
        break
    ble.stop_scan()
    


