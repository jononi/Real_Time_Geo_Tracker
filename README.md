# Real Time Geo Tracker

## Summary 
Real-time mobility tracker and logger with visualization dashboard and data storage. 
* Logged data: location, speed, heading, elevation, battery charge and sessions: traveled distance, travel time.
* Location/speed data can be received by a server running Influx DB to report location in real time on a map and save it to the database.
* Author: Jaafar Benabdallah
* Last updated: January 2019

* ![Dashboard example](/ressources/RealTimeTracker_Grafana_Dashboard.jpg?raw=true "Dashboard")

## Hardware
* Particle's Electron 2G 
* Sara Neo-M8N GNSS (GPS+Glonass) module (any breakout available at  Aliexpress will do)
* OLED 1.3" dislay (with SPI interface version) 
* accelerometer (using LSM6DS3 in this project) for G-force visualization and wake up on motion feature.
* Powerswtch to turn off OLED and GNSS modules in powersave mode. For example: [Mini MOSFET Slide Switch](https://www.pololu.com/product/2810) from Pololu.

## Software
* Electron's application firmware
* webhooks (to be provided soon)
* Influx DB, Influx telegraf, Grafana, Nginx webserver for secure proxy all running on a Raspberry Pi 3B if going with locally hosted platform (configuration files will be added soon). Cloud implementation is possible too.


## Version History
* v 0.17 -> log location(+geohash)/speed/altitude to influxdb via node.js service
* v 0.18 -> fixed keep alive feature when using Google Fi sim card
* v 0.2 -> added oled display, use of software timers for publishing data and refreshing display
* v 0.3 -> using ubx messaging protocol to communicate with GNSS module. Added software watchdog.
* v 0.31 -> dropped usage of software timers for tracking data publishing and display refreshing. (was not thread safe)
* v 0.4 -> enabled switched 3.3v supply for oled display + gnss module
* v 0.41 -> fixed minor bugs, watchdog time out period to 5 min
* v 0.42 -> reverted to Particle's data plan
* v 0.43 -> added trip time on a new bottom status bar. Removed particle.connect() blocking calls in loop()
* v 0.5 -> added IMU unit, added SLEEP STATE with network on standby, trip distance calculation and display
* v 0.6 -> added gball display for longitudinal and lateral acceleration, heading display
* v 0.61 -> now uploads latitude, longitude, speed (mph) and heading (degrees). Discarded: session_id and altitude
* v 0.62 -> changed pin mapping as in the snapshot: new_pin_mapping.jpg, changed libraries to reflect different SPI assignments
* v 0.63 -> changed the 3.3v switched power control out of gnss class and back to main sequence
* v 0.7 -> added high pass filter to remove gravity acceleration offset from z-axis acceleration when the device is tilted (when positioned on car dashboard for example)
* v 0.8  -> added wake on motion, sleep on no motion for > 2 min
* v 0.81 -> tweaked threshold for wake up. Track on autostart now works only at the first GNSS fix after a reset.
* v 0.82 -> fix when to resume session and when to reset it: by 3 clicks or on demand with cloud function StandBy_OnMove(r)
* v 0.83 -> add webhook to forward published data to telegraf (influx) instance and then to influxdb (to replace particle2influx server app?)
* v 0.84 -> fix session_id assignment after reset: wait for connection then assign it.
* v 0.85 -> fix blocking when cellular connection is lost.
* v 0.86 -> use Rickkas publishQueue when sending tracking and session data, without timestamp
* v 0.862 -> turn on tracking when speed > 7 mph
* v 0.863 -> fix odometer/trip time increment rules (now based on delta_distance > threshold)
* v 0.87 -> back to using Google Fi data sim/plan, various bug fixes, testing GNSS stopped mode without switching off SMPS-> abondoned for now
* v 0.88 -> parse new message from gnss: NAV-STATUS --> fix type and ttff. Added feature: get publish queue size
* v 0.89 -> reduce published data size + include time by creating a webhook to expand data to telegraf payload format
* v 0.9 -> software timers for imu reading and display refresh
* v0.91 -> [didnot work] fix when connecting (flashing green), a button press crashes Electron (assertion failure) -> local/global/int context confusion?
* v0.92 -> use of low pass filter to get acceleration bias then substarct it from current reading (more useful that high pass filter)
* v0.93 -> revert to 0.8.0-rc.10 in hope of avoiding assert failure panic on button push.
* v0.94 -> updated OS to 0.8.0-rc.12 -> (finally!)fixed crash when pressing the button in non listening mode. Extend autosleep timer to 2:30
* v0.95 -> added feature to cycle modem power with 4 clicks of MODE button: useful when modem is stuck when cloud connecting after a wake up
* TESTING v0.96 -> upgraded system firmware (now DeviceOS) to 1.0.0 
* **all future upgrades from here will be implemented on Boron version first **
* future: v 1.0alpha -> porting on Boron (hardware changes: replace 3.3V SMPS with a switch supplied by Boron's 3v3 output, display and imu on same SPI bus)
* future: v 1.1 -> enable 5Hz mode on GNSS, use Rickkas SerialBuffer lib and average GNSS fixes (study impact on power usage)

## Credits
* [Rick K](https://github.com/rickkas7) for finite state machine idea, power save modes description and publish queue library.
* [Ankur Dave](https://github.com/ankurdave) for writing ubx-protocol library for NEO M8 modules -> [project repo](https://github.com/ankurdave/IntervalTracker).