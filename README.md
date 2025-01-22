# FOUR-WHEELED OMNI BOT
![sexy omnibot](omnibot.jpg)
This is a **work in progress** project for a four-wheeled omni-directional robot. It’s powered by a **Raspberry Pi Pico W**, using **two L298N motor drivers** and **four CQRobot Ocean 131.3:1 Metal DC Geared-Down Motors**. The motors drive custom **omni wheels** you can find here: [Thingiverse](https://www.thingiverse.com/thing:2506412).

---

## Hardware Used

- **Raspberry Pi Pico W**: The brain of the bot.
- **Motor Drivers**: 2 × L298N.
- **Motors**: 4 × CQRobot Ocean 131.3:1 Metal DC Geared-Down Motors.
- **Omni Wheels**: Custom 3D-printed omni wheels from [Thingiverse](https://www.thingiverse.com/thing:2506412).

---

## What It Does (or Will Do)

- Moves in **any direction** (forward, sideways, diagonally, you name it but it doesn't fly).
- Wi-Fi control is planned (thanks to the Pico W).
- Uses PID control for smooth motor performance (eventually).

---

## Current Status

Right now, this is a skeleton project—everything is basic and constantly evolving. If you’re here expecting a polished product, this isn’t it yet.

---

## How to Use It

1. **Build the Hardware**:
   - Connect the motors to the L298N motor drivers.
   - Attach the omni wheels to the motor shafts.
   - Wire the L298N motor drivers to the Pico W.

2. **Get the Code Running**:
   - Flash MicroPython onto the Raspberry Pi Pico W.
   - Clone this repo and upload the files to the Pico W.
   - Make sure to update pin configurations in the code to match your setup.

3. **Power It On**:
   - Power the bot and run `main.py` to see it in action (or debug why it’s not working—this is a work in progress, remember?).

---

## Future Ideas

- **Add sensors**: Things like obstacle avoidance or mapping the environment.
- **Better control**: Fully remote control via Wi-Fi.
- **More features**: Add a raspbery pi  there are holes in the top to mount one a usb c powersuuply and a handy usb breakout it's almost like I planned this?

---

## Notes

If you’ve got suggestions, improvements, or just want to point out something broken, feel free to open an issue or submit a pull request.
