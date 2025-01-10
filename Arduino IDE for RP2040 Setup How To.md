# Arduino IDE Setup for Windows, Linux, and macOS with Raspberry Pi Pico (RP2040) Support

This guide shows you how to install the Arduino IDE and configure it for the **Raspberry Pi Pico (RP2040)**. We will install **both** the **Arduino Mbed OS** core and **Earle F. Philhower’s** core, so you have the flexibility to experiment with either option.

---

## 1. Installing the Arduino IDE

### Windows Installation
1. **Download the Installer**  
   - Go to the [Arduino Software page](https://www.arduino.cc/en/software).  
   - Under **"Downloads,"** choose the Windows installer (e.g., "Windows Win 7 and newer").

2. **Install the Arduino IDE**  
   - Double-click the downloaded `.exe` file.  
   - Follow the on-screen instructions.  
   - Choose default installation options unless you have a specific requirement.

3. **Launch Arduino IDE**  
   - After installation, open the **Arduino IDE** from the Start menu or desktop shortcut.

### Linux Installation
1. **Download the Installer**  
   - Go to the [Arduino Software page](https://www.arduino.cc/en/software).  
   - Select the "Linux 64-bit" (or 32-bit if you need it) package.

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
   - Drag and drop the **Arduino** app into your **Applications** folder.

3. **Launch Arduino IDE**  
   - Open **Finder > Applications**, then double-click **Arduino** to launch.  
   - You may need to approve the application under **System Settings > Security & Privacy** on the first launch.

---

## 2. Installing Raspberry Pi Pico (RP2040) Support

Below are instructions to install **both** the **Arduino Mbed OS RP2040 boards** and **Earle F. Philhower’s RP2040 boards**.

### 2.1 Installing Arduino Mbed OS RP2040

1. **Open the Board Manager**  
   - In Arduino IDE, go to **Tools > Board > Boards Manager**.

2. **Search for "mbed rp2040"**  
   - Type "`mbed rp2040`" or just "`rp2040`" into the search box.

3. **Install "Arduino Mbed OS RP2040 Boards"**  
   - Locate "**Arduino Mbed OS RP2040 Boards**" in the results and click **Install**.

4. **(Optional) Select the Board**  
   - Go to **Tools > Board**, scroll to **Arduino Mbed OS RP2040 Boards**, and select **Raspberry Pi Pico** if you want to try an Mbed OS sketch.

### 2.2 Installing Earle F. Philhower’s RP2040 Core

1. **Open Preferences**  
   - In Arduino IDE, go to **File > Preferences** (Windows/Linux) or **Arduino > Preferences** (macOS).

2. **Add Additional Board Manager URL**  
   - In the "**Additional Boards Manager URLs**" field, add:  
     ```
     https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
     ```
     If other URLs exist, separate them with a comma **`,`**.

3. **Open the Board Manager**  
   - Go to **Tools > Board > Boards Manager**.

4. **Search for "rp2040" or "Philhower"**  
   - Type "`rp2040`" or "`Philhower`" into the search box.

5. **Install "Raspberry Pi Pico/RP2040"**  
   - Locate "**Raspberry Pi Pico/RP2040 by Earle F. Philhower**" and click **Install**.

6. **Select the Board**  
   - Go to **Tools > Board**, scroll to "**Raspberry Pi RP2040 Boards**," and select "**Raspberry Pi Pico**" (or the variant you use).

---

## 3. Verifying Your Setup

You now have **both** Mbed OS and Earle F. Philhower’s RP2040 support installed. To confirm:

1. **Choose Which Core to Use**  
   - **Arduino Mbed OS RP2040**:  
     - **Tools > Board > Arduino Mbed OS RP2040 Boards > Raspberry Pi Pico**  
   - **Earle F. Philhower’s RP2040**:  
     - **Tools > Board > Raspberry Pi RP2040 Boards > Raspberry Pi Pico**  

2. **Select the Correct Port**  
   - Go to **Tools > Port** and select the port associated with your Raspberry Pi Pico.

3. **Load an Example**  
   - Try **File > Examples > 01.Basics > Blink** (or any simple sketch).

4. **Upload**  
   - Click the **Upload** button.  
   - If the process completes successfully without errors, your setup is correct.

> **Tip:** If you encounter issues uploading to the Pico via Arduino Mbed OS, try Earle Philhower’s core or vice versa. One core may work better depending on your environment and project requirements.

---

## 4. Additional Tips

- **Windows Driver Installation**  
  - Windows should automatically install drivers. If you have driver issues, refer to [Arduino Docs](https://docs.arduino.cc/) or the [Raspberry Pi Pico Getting Started Guide](https://www.raspberrypi.com/products/raspberry-pi-pico/).

- **Keep Cores Updated**  
  - Update your board packages in the Arduino IDE’s **Boards Manager** for the latest fixes and features.

- **Offline Installation**  
  - For offline installs, see the respective repositories or the Arduino docs for instructions.

- **Community Support**  
  - For troubleshooting, visit the [Arduino Forum](https://forum.arduino.cc/) or the [Raspberry Pi Forums](https://forums.raspberrypi.com/).

---
