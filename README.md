# ESP32_Lightbulb_switch
based on https://github.com/espressif/esp-apple-homekit-adk... this is how to get the switch to work and push notifications.

ConfigHardwareIO.c is over complicated for this use (supports configuring multiple pins at once) but its part of my next project.

there are a few flavors of ESP32 dev boards, i used https://www.amazon.com/dp/B09QW6Y7KY?th=1.
this board has a boot button connected to IO0 and a Blue LED connected to IO2

# compile and flash
Visual studio Code / ESP-IDE was used to compile this sample, and i wanted to skip manual steps. so if you want to skip 
```text
$ idf.py menuconfig # Set Example Configuration -> WiFi SSID/Password
$ idf.py flash
$ esptool.py -p $ESPPORT write_flash 0x340000 accessory_setup.bin
$ idf.py monitor
```
        
1- copy
    ../esp-apple-homekit-adk/examples/Lightbulb/accessory_setup.bin
     to 
    ../esp-apple-homekit-adk/examples/Lightbulb/build dir

--------------------------
2 - modified ../esp-apple-homekit-adk/examples/Lightbulb/build/flasher_args.json
    ... added two lines

in  "flash_files" 
 add->       "0x340000" : "accessory_setup.bin"

 new root item ->    "accessory_setup" : { "offset" : "0x340000", "file" : "accessory_setup.bin", "encrypted" : "false" },

----------------------------
3 - modified sdkconfig
    CONFIG_EXAMPLE_WIFI_SSID="LocalWIFI"
    CONFIG_EXAMPLE_WIFI_PASSWORD="LocalWIFI-PW"
