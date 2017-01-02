# VisionTest #
Playing with OpenCV Android vison for FRC

## Nexus 5 setup ##

The following enables the phone to boot and launch the vision app automatically on USB power (no user interaction needed)

### Prerequisites ###

- `adb` and `fastboot`. If you installed Android Studio, these are in the `sdk/platform-tools` folder of your Android SDK install.
- Adb drivers. https://developer.android.com/studio/run/win-usb.html Note: if task manager freezes on USB tethering, you'll need to install the windows rndis driver over the garbage version adb installs. Select vendor: "Microsoft" driver: "Remote NDIS compatible device" or something like that
- TWRP. https://dl.twrp.me/hammerhead/twrp-2.8.7.1-hammerhead.img
- ElementalX kernel. http://download1961.mediafire.com/7fwo77dgqtyg/b9am9gx9h6na9m7/N5-ElementalX-6.0.zip
- SuperSU. https://download.chainfire.eu/743/SuperSU/BETA-SuperSU-v2.52.zip

### Unlock the bootloader ###

If you see an unlocked icon on the first boot screen (under the Google logo), your phone is already unlocked, so skip this step. Do this before you install anything on the phone to avoid wasting time, as it wipes the internal storage.

- Connect the phone to your computer
- Power off, then hold power and volume down until you see a screen with a large green "Start" at the top and phone info on the bottom
- Start adb and make sure it sees the phone. `adb devices`
- Unlock the phone. `fastboot oem unlock`
- This takes a long time. Keep waiting until the phone fully reboots

### Enable developer mode ###

If you just unlocked, you'll want to set up the phone here.

- Go to Settings > About and tap the build code item at the bottom of the about screen a bunch of times
- Back on the main settings menu, go to the new developer page and enable USB debugging. Then authorize your computer

### Copy `N5-ElementalX-6.0.zip` and `BETA-SuperSU-v2.52.zip` to the phone ###

You can do it with `adb push` or enable MTP via the notifications tray

### Install TWRP ###

- Power off, power on + volume down to enter fastboot again
- `fastboot flash recovery "C:\path\to\twrp-2.8.7.1-hammerhead.img"`
- Here, to make the phone power on by itself when USB power is connected (so it starts on the field once the robot turns on, with no user interaction required), do `fastboot oem off-mode-charge 0`

### Root ###

- In fastboot, press the volume up key until "Recovery" is selected, then press power
- Tap "Install" then select the ElementalX kernel zip and swipe to flash it
- Go back to home, "Install", and select the SuperSU zip and swipe to flash it
- Reboot. Wiping isn't necessary here

### 🎉 You're done 🎉 ###

You may want to make SuperSU always grant root to avoid having to click the dialog each time you deploy a new version of the app