# Arduino IDE Setup for Windows, Linux, and macOS with Raspberry Pi Pico (RP2040) Support

This guide outlines how to install the Arduino IDE and configure it for the **Raspberry Pi Pico (RP2040)**. We will be using **Earle F. Philhower’s RP2040 boards** instead of the Arduino Mbed OS option.

---

## 1. Installing the Arduino IDE

### Windows Installation
1. **Download the Installer**  
   - Visit the [Arduino Software page](https://www.arduino.cc/en/software).  
   - Under "Downloads," choose the Windows installer (e.g., "Windows Win 7 and newer").

2. **Install the Arduino IDE**  
   - Double-click the downloaded `.exe` file.  
   - Follow the on-screen instructions.  
   - Choose default installation options unless you have a specific requirement.

3. **Launch Arduino IDE**  
   - After installation, open the Arduino IDE via the Start menu or desktop shortcut.

### Linux Installation
1. **Download the Installer**  
   - Visit the [Arduino Software page](https://www.arduino.cc/en/software).  
   - Select the "Linux 64-bit" (or 32-bit if needed) package.

2. **Extract and Install**  
   - Open a terminal and navigate to the folder where you downloaded the `.tar.xz` file.  
   - Extract it using:  
     ```bash
     tar -xvf arduino-<version>-linux64.tar.xz
     ```
   - Enter the extracted folder and run the `install.sh` script:  
     ```bash
     cd arduino-<version>
     sudo ./install.sh
     ```
   - This will create desktop and menu shortcuts.

3. **Launch Arduino IDE**  
   - Start the IDE from your Applications menu or by typing `arduino` in a terminal.

### macOS Installation
1. **Download the Installer**  
   - Go to the [Arduino Software page](https://www.arduino.cc/en/software).  
   - Select the macOS version (`.dmg`).

2. **Install the Arduino IDE**  
   - Double-click the downloaded `.dmg`.  
   - Drag and drop the Arduino app into your **Applications** folder.

3. **Launch Arduino IDE**  
   - Open **Finder > Applications**, then double-click **Arduino** to launch.  
   - You may need to approve the application under **System Settings > Security & Privacy** on first launch.

---

## 2. Installing Raspberry Pi Pico (RP2040) Support (Earle F. Philhower’s Core)

> **Note:** We will not use the **Arduino Mbed OS** option. Follow the steps below to install **Earle F. Philhower’s RP2040 boards**, which provide robust community support and additional features for the Raspberry Pi Pico.

1. **Open Preferences**  
   - In Arduino IDE, go to **File > Preferences** (Windows/Linux) or **Arduino > Preferences** (macOS).

2. **Add Additional Board Manager URL**  
   - In the "**Additional Boards Manager URLs**" field, add the following URL:  
     ```
     https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
     ```
     If there is already text in this field, place a comma **`,`** between existing entries and the new URL.

3. **Open the Board Manager**  
   - Go to **Tools > Board > Boards Manager**.

4. **Search for "rp2040"**  
   - Type "`rp2040`" or "`Philhower`" into the search box.

5. **Install the "Raspberry Pi Pico/RP2040" Core**  
   - Locate "**Raspberry Pi Pico/RP2040 by Earle F. Philhower**" and click **Install**.

6. **Select the Board**  
   - Go to **Tools > Board**, scroll to "**Raspberry Pi RP2040 Boards**," and select "**Raspberry Pi Pico**" (or whichever variant you are using).

7. **Connect and Test**  
   - Connect your Raspberry Pi Pico to your computer using a USB cable.  
   - In the Arduino IDE, select the correct serial port under **Tools > Port**.  
   - Test by uploading a simple example (e.g., **File > Examples > 01.Basics > Blink**) to confirm that everything works properly.

---

## 3. Verifying Your Setup

1. **Select the Correct Board**  
   - Under **Tools > Board**, ensure you have chosen your RP2040 board from "**Raspberry Pi RP2040 Boards**."

2. **Select the Correct Port**  
   - Go to **Tools > Port** and pick the port matching the connected Raspberry Pi Pico.

3. **Load an Example**  
   - Try **File > Examples > 01.Basics > Blink** (or any simple sketch).

4. **Upload**  
   - Click the **Upload** button.  
   - Watch the console output; the upload should complete without errors.

---

## 4. Additional Tips

- **Windows Driver Installation**  
  - Windows usually installs the necessary Pico drivers automatically. If you have issues, consult the [Arduino Docs](https://docs.arduino.cc/) or the [Raspberry Pi Pico Getting Started Guide](https://www.raspberrypi.org/documentation/microcontrollers/).

- **Keep Cores Updated**  
  - Update your board packages in the Arduino IDE’s **Boards Manager** periodically for the latest fixes and features.

- **Offline Installation**  
  - If you want to install board packages manually (offline), refer to the Earle F. Philhower repo README or the Arduino Docs for detailed instructions.

- **Community Support**  
  - For troubleshooting, visit the [Arduino Forum](https://forum.arduino.cc/) or the [Raspberry Pi Forums](https://forums.raspberrypi.com/).

---

## 5. Conclusion

By following these steps, you’ll have the Arduino IDE set up on your preferred operating system and configured for the **Raspberry Pi Pico (RP2040)** using **Earle F. Philhower’s** core. You can now write, compile, and upload sketches to your Pico, leveraging the rich Arduino ecosystem. Happy building!
