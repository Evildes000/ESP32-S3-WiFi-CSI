# ESP32-S3-WiFi-CSI
This repo gives you code that can be used to record WiFi CSI by using ESP32-S3

# How to use it? 
1. Install ESP-IDF
2. Open installed ESP-IDF and navigate to where you downloaded this repo
3. Execuate following command:

   idf.py set-target esp32s3.  
   idf.py build.
   idf.py -p COM<your port number> flash.
   the port number can be found in device manager.
4. Now esp32 starts to record CSI data
5. type command:
   esptool.py -p COM10 -b 460800 read_flash 0x310000 0x200000 csi_dump.bin.
   to dump recorded CSI data in binary file.
6. run:
   python parse_csi.py csi_dump.bin --plot.
   to plot a CSI data.
