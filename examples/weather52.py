# Weather display using a rtbobble52
import time
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from bs4 import BeautifulSoup
import requests

def getWeather():
    #use google to get the weather for a location 
    location=input("Enter location:")
    if len(location.strip())==0:
        location="San Mateo ca"

    location= location.replace(" ","+")

    print ("Location: "+location)


    soup = BeautifulSoup(requests.get("https://www.google.com/search?q=weather+"+location).content,'html.parser')
    temp=soup.find("div", class_="BNeawe tAd8D AP7Wnd")
    climate=temp.text.title()
    print (climate)
    servo =0

    #set the servo based on the weather
    if "Sunny" in climate or "Clear" in climate:
        servo=150

    if "Cloudy" in climate:
        servo=120

    if "Fog" in climate or "Haze" in climate:
        servo=90

    if "Rain" in climate:
        servo=60

    if "Snow" in climate:
        servo=30
        
    print ("servo="+str(servo))
    return str(servo)

#Main code starts here

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
                #get weather info
                servo= getWeather()

                #convert the data into a bobble52 command
                bobble_command = servo+' s'
                print(bobble_command)

                #output the data to the bobble52
                uart.write(bobble_command.encode("utf-8"))
                uart.write(b'\n')


    print("disconnected, scanning")
    for advertisement in ble.start_scan(ProvideServicesAdvertisement, timeout=1):
        if UARTService not in advertisement.services:
            continue
        ble.connect(advertisement)
        print("connected")
        break
    ble.stop_scan()
    




