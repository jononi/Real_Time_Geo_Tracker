/*
* Project: real time mobility tracker and logger
* Description: location/speed tracker and logger based on Particle's Electron and Neo M8 GNSS module.
* Location/speed data can be received by a server running Influx DB to report and save location in real time on a map.
* Author: Jaafar Benabdallah
* Last updated: January 2019
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
* future: v 1.2 -> add joystick to augment MODE button functions + simple menus to display additional info (temperature, gnss stats, publish stats...) 
* 
* Credits: Rick K(rickkas7) for FSM, power save modes and publish queue, Ankur Dave for ubx-protocol library for NEO M8 modules
*/

// to avoid compile error when building for photon
#if ( PLATFORM_ID == PLATFORM_ELECTRON_PRODUCTION)
#define cellular
#endif

// enable serial log
// #define serial_debug
// use google fi sim
#define googlefi

#include "SparkFunLSM6DS3.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SH1106_Particle.h"
#include "ubx_neom8n.h"
#include "math.h"

#ifdef cellular
//#include "Serial5/Serial5.h"
#include	"cellular_hal.h"
#include	"PublishQueueAsyncRK.h"
#endif

//peripherals control pins mapping
#define OLED_CS     B2	// /SS 128x64 OLED (in addition to SCK, MOSI on SPI)
#define IMU_CS		C0
#ifdef cellular
#define OLED_DC     B1
#define OLED_RESET  B0
#define V33_SW		B5
#else
#define OLED_DC     D1
#define OLED_RESET  D0
#define V33_SW		D6
#endif

// Time bases for periodic events
const unsigned long KEEPALIVE_PERIOD 	= 30; // in seconds, when using google fi sim
const unsigned long DISPLAY_PERIOD 		= 300; // display refresh (ms)
const unsigned long ODOMETER_PERIOD 	= 1000; // trip stats update (ms)
const unsigned long IMU_PERIOD 			= 100; // acceleration data update (ms)
	  unsigned long PUBLISH_PERIOD 		= 20000; // default publish period (ms)
const unsigned long AUTOSLEEP_TIME		= 150 * 1000; // inactivity time (ms) before going in sleep mode
const unsigned long LED_PERIOD 			= 2000; // tracking on indicator (ms)
#ifdef googlefi
const unsigned long SLEEPING_TIME 		= 60 * 60; // wakeup every 120 min (have to reconnect with Fi anyway)
#else
const unsigned long SLEEPING_TIME 		= 22 * 60; // wakeup every 22min if no motion detected (keep network connection alive)
#endif

// Tracking parameters
const double mm_per_second_per_mph	= 447.04; // mm/s -> mph
	  double delta_dist_th			= 10.0; // min change in distance (meters) between locations to increment odometer
const double alpha 		=	0.9; // high pass filter param
const double beta		= 	0.2; // low pass filter (MA) first order coeff
const double motion_th	=	0.12; // min accel magnitude in g to trigger/maintain wake up

// Tracking settings
uint32_t	session_id;
bool		session_id_reset	= true;  // reset session_id flag
bool		reset_session_flag	= false; // reset session flag
bool		tracking_on_flag	= false;
bool 		assistNow_on_flag	= false; // still buggy: GNSS Assist Now feature

// Operations variables
bool	enter_standby_flag  = false; // Power saving state flag
bool	cloud_conn_reqed	= false; // to avoid repeated calls to Particle.connect()
bool 	cycle_modem_power_flag = false;
volatile int		nb_clicks	= 0;

// Timing variables
uint32_t	last_updated_odometer	= 0; // elapsed time in ms since last trip time/distance update
uint32_t	last_tracking_publish 	= 0;
uint32_t	publish_extra_time 		= 0;	//to alter publish period if no fix or very low speed
uint32_t	last_refresh_display	= 0;
uint32_t 	update_start_time		= 0;

// GNSS stats
uint32_t 	gnss_begin_ms = 0;
uint32_t 	ttff = 0;
uint32_t 	gnss_last_fix_ms = 0;

// GNSS data
bool		gnss_on = false;
bool		valid_fix_flag = false;
double		lat = 0.0, lon = 0.0, alt = 0.0, hAcc = 0.0, speed_mph = 0.0, heading = 0.0;
double		old_lat = 0.0, old_lon = 0.0, delta_dist = 0.0; // used to calculate traveled distance
uint8_t 	nb_sats = 0;
uint8_t 	fixType = 0;
uint32_t	tripTime = 0; // elapsed time in s since last tracking session
double  	tripDistance_m = 0.0; // distance in m since last tracking session

// IMU data
bool 	imu_on = false; // status flags
bool	update_start_on = false; // ready to be set when vehicle stops
float 	accel[3]; // acceleration vector 
float 	old_filt_acc_z = 0.0, old_acc_z = 0.0, acc_z = 0.0;
float	acc_z_buff[3];
float	a0_y = 0.0, a0_z = 0.0, acc_mag = 0.0;

// Text buffer
char 	buffer[256];
// publish queue buffers
retained uint8_t publishQueueRetainedBuffer[2560]; //~21 points
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));

// FSM states
enum State {CONNECT_WAIT_STATE, TRACKING_OFF_STATE, TRACKING_ON_STATE, SLEEP_STATE};
State state;

#ifdef serial_debug // Serial debug configuration
SerialLogHandler logHandler(115200, LOG_LEVEL_WARN, {
	{ "app", LOG_LEVEL_INFO },
	{ "app.pubq",LOG_LEVEL_TRACE }
});
#endif

// Required forward declarations
void gnssMessageHandler(uint16_t msg_class_id, const ubx_buf_t &buf); //@rx valid gnss msg
void wd_callback() { System.sleep(10);} // turn off cellular module for 10s

// Global instances
UBX_NEOM8N	gnss(gnssMessageHandler);
LSM6DS3 	imu(SPI_MODE, IMU_CS);
#ifdef cellular
FuelGauge	battery;
PMIC 		pmic; // to change PMIC settings
#endif
Adafruit_SH1106 display(OLED_DC, OLED_RESET, OLED_CS);
ApplicationWatchdog wd(300000, wd_callback); // called after 5 min no connection

// Software timers
Timer trackingLEDTimer(LED_PERIOD, trackingLEDUpdate);
Timer autoSleepTimer(AUTOSLEEP_TIME, autoSleepHandler);
Timer watchdogTimer(5000, wd_check_in);
Timer imuTimer(IMU_PERIOD, imu_update);
// Timer displayTimer(DISPLAY_PERIOD, display_update);

// Startup options configurtion
#ifdef googlefi
STARTUP(cellular_credentials_set("h2g2", "", "", NULL));
#endif
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);


void setup() {
	#ifdef cellular
	battery.wakeup();
	battery.quickStart();
	pmic.setChargeVoltage(4208);  // set charging voltage up to 4.208V --> will charge to 100% (if batt. doesn't overheat)
	#endif

	//system events
	// register SETUP button event handler
	System.on(button_final_click, modeButtonHandler);
	// report cloud event to console
	#ifdef serial_debug
	System.on(cloud_status, cloud_statusHandler);
	#endif

	// display setting up
	display.begin(SH1106_SWITCHCAPVCC);
	display.clearDisplay();
	// display splash screen
	display.drawBitmap(0, 4, particleLogo, 128, 48, 1);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(15,54);
	display.print("Real-Time Tracker");
	display.display();

	// start up the gnss module
	gnss_begin_ms = millis();
	gnss_last_fix_ms = millis();
	resetGNSSInfo();
	gnss_on = gnss.start(&Serial1, 9600, V33_SW);

	// configure and start up the IMU module
	imu_on = init_imu();

	// register cloud functions before calling Particle.connect()
	Particle.function("Set_Tracking", startTracking);
	Particle.function("Get_Battery", batteryStatus);
	Particle.function("Get_Status", getStatus);
	Particle.function("Get_Location", trackingDataPublish);
	Particle.function("Sleep_WakeUp_OnMove", enter_standby);

	// blue LED on D7 is used to indicate tracking is on
	pinMode(D7, OUTPUT);
	digitalWrite(D7, LOW);

	// adjust local time
	Time.zone(-5);
	Time.setDSTOffset(1.0);
	// Time.beginDST(); // uncomment when it's daylight saving time

	// initial screen
	display.clearDisplay();
	display.setTextSize(1);
	// top status bar
	display.setCursor(48,0);
	display.printf("%02d:%02d", Time.hour(), Time.minute());
	#ifdef cellular
	display.setCursor(104, 0);
	display.printf("%.0f%%", battery.getSoC());
	#endif
	display.drawFastHLine(0, 9, display.width(), WHITE);
	// startup info
	display.setCursor(6, 10);
	display.printf("gnss init. %s..",gnss_on?"ok":"failed");
	// bottom status battery
	display.drawFastHLine(0, 55, display.width(), WHITE);
	display.setCursor(0, 57);
	display.print("trip time / distance");
	display.display();

	// initiate cellular + cloud connection
	Particle.connect();

	#ifdef serial_debug
	Serial.begin();
	delay(2000);
	#endif

	// initial state after reset
	state = TRACKING_OFF_STATE;
	watchdogTimer.start();
	autoSleepTimer.start();
	imuTimer.start();
	// displayTimer.start();
	last_refresh_display = millis();
}


void loop() {
	// FSM transitions
	switch(state) {
		case CONNECT_WAIT_STATE:
		if (enter_standby_flag) {
			state = SLEEP_STATE;
		}
		if (Particle.connected()) {
			if (tracking_on_flag) {
				state = TRACKING_ON_STATE;
				trackingLEDTimer.start();
			} else {
				state = TRACKING_OFF_STATE;
			}
		}
		break;

		case TRACKING_OFF_STATE:
		if (enter_standby_flag) {
			state = SLEEP_STATE;
		}
		if (!Particle.connected()) {
			// cloud connection lost
			#ifdef serial_debug
			Log.info("retrying to connect...");
			#endif
			state = CONNECT_WAIT_STATE;
		}
		if (tracking_on_flag) {
			#ifdef serial_debug
			Log.info("switch tracking on");
			#endif
			state = TRACKING_ON_STATE;
			trackingLEDTimer.start();
		}
		break;

		case TRACKING_ON_STATE:
		if (enter_standby_flag) {
			state = SLEEP_STATE;
		}
		if (!Particle.connected()) {
			// cloud connection lost
			#ifdef serial_debug
			Log.info("retrying to connect...");
			#endif
			state = CONNECT_WAIT_STATE;
			trackingLEDTimer.stop();
			digitalWrite(D7, LOW);
		}
		if (!tracking_on_flag) {
			#ifdef serial_debug
			Log.info("switch tracking off");
			#endif
			state = TRACKING_OFF_STATE;
			trackingLEDTimer.stop();
			digitalWrite(D7, LOW);
		}
		break;

		case SLEEP_STATE:

		/* Pre Sleep Routine */
		pre_sleep_sequence();

		/* Rickkas did an interesting analysis on which standby mode is best for different scenarios
		* https://github.com/rickkas7/electron-power-data-usage
		* SLEEP_NETWORK_STANDBY option is a 6.53mA on average standby mode but no need to reset cellular connection.
		* It's a faster wakeup and it saves energy by reducing radio transmission.
		* another Rickkas trick: remote wake up using modem's incoming data trigger:
		* https://community.particle.io/t/wakeon-ri-in-sleep-network-standby/43756/3
		*/
		//Not Working: it's available to 3G modem only. Enable wake up on all URCs. We'll see with Boron
		//Cellular.command("AT+URING=1\r\n");delay(1000); 
		System.sleep(WKP, RISING, SLEEPING_TIME, SLEEP_NETWORK_STANDBY);

		/* Post Wake up routine */
		post_awake_sequence();
		state = CONNECT_WAIT_STATE;
		break;
	} //FSM transitions

	/*
	*    LOOP() other tasks
	*
	*NOTE
	*Keep Alive actions is not needed if using Particle Sim card (default value of 23 min).
	*Since firmware 0.8.4, Particle.keepAlive() behaviour changed regarding requirement of cloud connection.
	*but for the time being, I still wait for cloud connection before changing it
	*/

	#ifdef googlefi
	// one-time action that needs to run after cloud connection
	static bool keepAlive_set = false;

	if (!keepAlive_set && Particle.connected()) {
		Particle.keepAlive(KEEPALIVE_PERIOD);
		keepAlive_set = true;
		#ifdef serial_debug
		Log.info("keepalive period updated to %lu seconds.", KEEPALIVE_PERIOD);
		#endif
	}
	#endif

	// a one-time check to set session_id after reset
	if (Time.isValid() && session_id_reset) {
		session_id = Time.local();
		session_id_reset = false;
	}

	// in Tracking On mode, check if it's time to publish GNSS data
	if (tracking_on_flag && millis() > (last_tracking_publish + PUBLISH_PERIOD + publish_extra_time)) {
		trackingDataPublish("");
		last_tracking_publish = millis();
	}

	// in Tracking On mode, check if it's time to increment trip distance/time
	if (tracking_on_flag && millis() > (last_updated_odometer + ODOMETER_PERIOD)) {
		// calculate distance if there's significant change
		if (valid_fix_flag) {
			if (old_lat == 0) {
				// first time with valid fix, save initial location
				old_lat = lat;
				old_lon = lon;
			} else {
				delta_dist = distance_angle_between(old_lat, old_lon, lat, lon, true);
				// delta distance threshold is max(horizontal_accuracy, 10) in meters
				delta_dist_th = hAcc > 10.0 ? hAcc : 10.0;
				if (delta_dist > delta_dist_th) {
					// increment distance only when delta > threshold to reduce inaccuracy due to location nois
					#ifdef serial_debug
					Log.info("d_lat=%.6f", lat - old_lat);
					Log.info("d_lon=%.6f", lon - old_lon);
					Log.info("d_dist=%.2f m", delta_dist);
					#endif
					tripDistance_m = tripDistance_m + delta_dist;
					// save previous location only when there was significant change in location
					// useful when in low speed (walking) it takes more time to get significant delta distance
					old_lat = lat;
					old_lon = lon;
				}
			}
		}
		// trip time is updated as long as tracking is on (regardless of fix validity)
		tripTime += (millis() - last_updated_odometer) / 1000;
		last_updated_odometer = millis();
	}

	
	if (millis() > (last_refresh_display + DISPLAY_PERIOD)) {
		display_update();
		last_refresh_display = millis();
	}


	/* get assist now data provided by u-blox. Requires an HTTP request and a valid token provided by u-blox
	* try it once or every 10 sec while waiting for a fix
	* blocks for up to 10 sec
	* TODO do it in a non blocking way? --> get the while loop out of here or run in its own thread?
	*/
	if (!valid_fix_flag && assistNow_on_flag) {
		gnss.assist(); //blocks for up to 10 sec, needs cloud connection
		assistNow_on_flag = false;
	}

	// cycle power of cellular modem to force it to re-establish a clea connection with network
	if (cycle_modem_power_flag) {
		cycle_modem_power_flag = false;
		Cellular.off();
		delay(3000);
		Cellular.on();
		Cellular.connect();
	}


} //loop


/* this runs in between calls to loop() to
* feed gnss parser with available data on the UART */
void serialEvent1() {
		gnss.update();
}


void pre_sleep_sequence() {
	//show sleep icon and then disable display module
	// displayTimer.stop();
	display.clearDisplay();
	display.drawBitmap(39, 7, power_sleep_icon, 48, 48, 1);
	display.display();

	// reset session if requested. Clear flag only if successful
	if (reset_session_flag) {
		reset_publish_session();
		reset_session_flag = false; // clear flag if reset successful
		// experimental: allow some time to make sure the publish go through
		delay(1000);
	}

	//digitalWrite(OLED_DC, LOW); // keeping OLED_DC high is a nice hack to maintain GNSS in standby mode
	// software shutdown gnss module then power off display and gnss modules
	gnss.stop();
	// clear display
	display.clearDisplay();
	display.display();
	digitalWrite(OLED_CS, HIGH);
	digitalWrite(V33_SW, LOW);
	#ifdef serial_debug
	Log.info("Going into network standby sleep mode");
	#endif
	if (Particle.connected()) {
		Particle.publish("GNSS/sleep", Time.format(Time.now(), "%FT%T"), 60, PRIVATE); //Time.timeStr()
	}
	// enable wake up on motion interrupt of the imu module
	enable_WakeOnMotionInterrupt();
}


void post_awake_sequence() {
	enter_standby_flag  = false;
	disable_WakeOnMotionInterrupt();
	// reset GNSS module
	gnss.start(&Serial1, 9600, V33_SW); //TODO replace with a function that does resume() rather than (re)start()
	resetGNSSInfo();
	// resume display activity
	display.begin(SH1106_SWITCHCAPVCC);
	display.display();
	// displayTimer.reset();
	SleepResult r = System.sleepResult();
	if (r.wokenUpByPin()) {
		#ifdef serial_debug
		Log.info("The device was woken up by pin %lu", r.pin());
		#endif
	} else if (r.wokenUpByRtc()) {
		batteryStatus(""); // post battery percentage when waking up because of timeout
		#ifdef serial_debug
		Log.info("The device was woken up by RTC");
		#endif
	}
	// if resume in tracking on mode, substract the time when there was no motion
	if (tracking_on_flag && tripTime > (AUTOSLEEP_TIME / 1000) )  {
		tripTime -= (int)(AUTOSLEEP_TIME / 1000);
	}
	last_updated_odometer = millis();
	// re-allow update of acceleration bias if no motion
	update_start_on = false;
}


 // IMU handler: get current acceleration and reset auto sleep timer if there's motion
void imu_update() {
	
    accel[0] = imu.readFloatAccelX();
	accel[1] = imu.readFloatAccelY();
	accel[2] = imu.readFloatAccelZ(); 

	// get the magnitude
	acc_mag = sqrt(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	
	// get the deviation from acceleration at rest (==1g at any orientation)
	// acc_mag = 1.0 - acc_mag;
	acc_mag = fabs(1.0 - acc_mag);

	/* at any orientation, @rest: ||a|| = 1 => abs(1 - ||a||) < eps is a good criterion
	* for no change in motion.
	* reset autosleep timer if there's a change of speed or a good shake */
	if (acc_mag > motion_th) {
		autoSleepTimer.reset();
    }

	// improved LOW-PASS filter approach
	// 1- when vehicle is stopped (residual vibration and speed ~ 0), trigger the start of updating the acceleration bias due to tilt
	if (acc_mag < 0.005 && speed_mph < 2.0 && !update_start_on) {
		update_start_on = true;
		update_start_time = millis();
	}
	// 2- for the next 2 seconds, update the acceleration bias (20 times)
	if ( (millis() - update_start_time) < 2000) {
		a0_y = a0_y + beta*(accel[1] - a0_y);
		a0_z = a0_z + beta*(accel[2] - a0_z);
	} else {
		//3- at the expiration of 2 seconds, wait until there's gain of speed to allow for another update period 
		if (speed_mph > 10.0) update_start_on = false;
	}

	/* the tilt-compensated longitudinal acceleration is:
	*  a_z = (accel[2] - accel[2]_rest) / -1*accel[1]_rest
	*  when device is upright, accel[2]_rest = 0 and accel[1]_rest = -1 
	*/
	accel[2] = (a0_z - accel[2]) / a0_y; // calculate the compensated longitudinal acceleration

	// accel[2] = accel[2] + beta*(accel[2] - accel2_old); // MA order 1  

	// HIGH-PASS filter approach
	// compensating for tilt on the Z-axis by offesetting the projection of g-vector
    /*acc_z =  accel[2]; //a_z[i] := x[i]
    // using a high pass filter
    accel[2] = alpha*(old_filt_acc_z + acc_z - old_acc_z); // y[i] = alpha*( y[i-1] + x[i] -x[i-1] )
    old_filt_acc_z = accel[2]; //y[i-1]
    old_acc_z = acc_z; //x[i-1]
	*/
}

/*
*		GNSS data publish function: called either in tracking on mode or on demand as a cloud function
*/
int trackingDataPublish(String command) {
	int exit_code =  -1;
	int fix_age_ms = millis() - gnss_last_fix_ms;
	#ifdef serial_debug
	Log.info("publish gnss data requested");
	Log.info("gnss on since: %lu ms", millis() - gnss_begin_ms);
	Log.info("latest tttf: %lu ms", ttff);
	Log.info("fix age: %lu ms, validity: %d", millis() - gnss_last_fix_ms, valid_fix_flag);
	#endif

	if (!valid_fix_flag) {
		snprintf(buffer, sizeof(buffer), "%d s", (int)(fix_age_ms / 1000));
		Particle.publish("GNSS/no fix", buffer, 60, PRIVATE);
		// incease publish period since we don't need a "no fix" every 10s
		publish_extra_time = 40000;
		return exit_code;
	}

	if ((valid_fix_flag && fix_age_ms < 10000) || (fixType == 3)) {
		const char *command_c = command.c_str();
		if (strcmp(command_c, "srv") == 0 ) {
			// publish -> nodejs -> influxdb (old way)
			const char *pattern = "{\"session_id\":\"%lu\",\"lat\":%f,\"lon\":%f,\"spd\":%.1f,\"hd\":%.0f,\"alt\":%.0f}";
			snprintf(buffer, sizeof(buffer), pattern, session_id, lat, lon, speed_mph, heading, alt);
			Particle.publish("GNSS/srv",buffer, 60, PRIVATE);
		} else {
			// get current local timestamp and shift back to UTC
			time_t utc = Time.now() + 5*3600;
			// utc += Time.isDST()?4*3600:5*3600;
			const char *timeString = Time.format(utc, "%FT%T").c_str(); //format date/time string as expected by telegraf Particle'splugin
			const char *pattern = "{\"id\":%lu,\"la\":%f,\"lo\":%f,\"sp\":%.1f,\"hd\":%.0f,\"el\":%.0f,\"ts\":\"%s\"}";
			snprintf(buffer, sizeof(buffer), pattern, session_id, lat, lon, speed_mph, heading, alt, timeString);
			publishQueue.publish("GNSS/data",buffer, 60, PRIVATE);
		}
		publish_extra_time = 0;
	}

	if (valid_fix_flag && fix_age_ms >= 10000) {
		snprintf(buffer, sizeof(buffer), "%d s",(int)(fix_age_ms / 1000));
		Particle.publish("GNSS/obsolete", buffer, 60, PRIVATE);
		publish_extra_time = 40000;
	}

	exit_code =  millis() - gnss_last_fix_ms;
	return exit_code;
}


/* remotely start/stop tracking and tracking period
* command = 0..10 --> tracking off
* command >= 10 and < 1800 --> tracking on, published every command seconds
*/
int startTracking(String command) {
	unsigned long _period = command.toInt();
	if (_period >= 10 && _period < 1800) {
		tracking_on_flag = true;
		PUBLISH_PERIOD = _period*1000;
		last_updated_odometer = millis();
	}
	else {
		// any other value will stop real time tracking and logging
		tracking_on_flag = false;
		PUBLISH_PERIOD = 20000; // for serial debug
	}
	return _period;
}


int batteryStatus(String command){
	// Publish the battery percentage remaining
	#ifdef cellular
	float batterySoc = battery.getSoC();
	#else
	float batterySoc = 100.0;
	#endif
	//new format for telegraf webhook:
	snprintf(buffer, sizeof(buffer), "{\"tags\":{\"id\":\"%lu\"},\"values\":{\"percentage\":%.2f}}", session_id, batterySoc);
	#ifdef serial_debug
	Log.info(buffer);
	#endif
	publishQueue.publish("GNSS/bat", buffer, 60, PRIVATE);
	// if (Particle.connected()) { Particle.publish("GNSS/bat", buffer, 60, PRIVATE); }
	return (int)batterySoc;
}


int getStatus(String command) {
	char state_label[16];
	int exit_code;
	if (state == TRACKING_ON_STATE) {
		strcpy(state_label, "tracking on");
		exit_code = (int)((PUBLISH_PERIOD + publish_extra_time) / 1000);
	}
	else if (state == TRACKING_OFF_STATE) {
		strcpy(state_label, "tracking off");
		exit_code = 0;
	}
	else {
		return -1;
	}
	Particle.publish("GNSS/status", state_label, 60, PRIVATE);
	return exit_code;
}


int enter_standby(String command) {
	const char *command_c = command.c_str();
	// deep sleep
	if (strcmp(command_c, "off") == 0 || strcmp(command_c, "d") == 0 ) {
		/* this is the same mode called by a double-click on SETUP button, but it's
		* limited in duration (8h).
		* it's 110uA on average with all peripherals on stand by
		* Device will reset when exiting this state
		*/
		System.sleep(SLEEP_MODE_SOFTPOWEROFF, 60*60*8);
		return 2;
	} else if (strcmp(command_c, "dfu") == 0 ) {
		System.dfu(); // enter DFU mode to allow new program flashing via usb
		return 4;
	} else if (strcmp(command_c, "r") == 0 ) {
		tracking_on_flag = false;
		reset_session_flag = true; // go for reset session id
		enter_standby_flag = true; // go to power save mode
		return 3;
	} else if (strcmp(command_c, "rid") == 0 ) {
		reset_session_flag = true; // go for reset session id
		return 1;
	} else {
		enter_standby_flag = true;	// go to power save mode
		return 0;	
	}
}


void gnssMessageHandler(uint16_t msg_class_id, const ubx_buf_t &buf) {
	switch (msg_class_id) {
		case UBX_MSG_NAV_PVT:
			valid_fix_flag = ((buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1);
			// update time of latest update
			if (valid_fix_flag) {
				gnss_last_fix_ms = millis();// keep it up to date unless fix is lost
			}
			// update tracking data
			lat = (double)buf.payload_rx_nav_pvt.lat * 1e-7;
			lon	= (double)buf.payload_rx_nav_pvt.lon * 1e-7;
			alt = (double)buf.payload_rx_nav_pvt.height * 1e-3;
			hAcc = (double)buf.payload_rx_nav_pvt.hAcc * 1e-3;
			nb_sats = buf.payload_rx_nav_pvt.numSV;
			speed_mph = (double)buf.payload_rx_nav_pvt.gSpeed / mm_per_second_per_mph;
			heading = (double)buf.payload_rx_nav_pvt.headMot * 1e-5;

			if (speed_mph > 7.0) {
				// autostart tracking
				if (!tracking_on_flag) {
					tracking_on_flag = true;
					last_updated_odometer = millis();
				}
				publish_extra_time = 0;
			} else {
				// slow speed, extend publish time by 30s
				publish_extra_time = 30000;
			}
			break;

		case UBX_MSG_NAV_STATUS:
			fixType = buf.payload_rx_nav_status.gpsFix;
			ttff 	= buf.payload_rx_nav_status.ttff;
			#ifdef serial_debug
			Log.info("fix type=%d", fixType);
			#endif
			break;

		default:
			valid_fix_flag = 0; // reset flag if no successful nav_pvt message parsing
	}
}


void modeButtonHandler(system_event_t event, int data) {
	//bug: calling this handler induce 'assert fail' crash
	nb_clicks = system_button_clicks(data);
	switch (nb_clicks) {
		// start / pause tracking
		case 1: {
			tracking_on_flag = !tracking_on_flag;
			#ifdef serial_debug
			Serial.printf("tracking status: %s\n",tracking_on_flag?"on":"off");
			#endif
			if (tracking_on_flag) last_updated_odometer = millis();
			break;
		}
		// end session and go into deep sleep mode (requires press on Reset button to wake up)
		case 2: {
			uint32_t trip_minutes = (tripTime > 60)? tripTime / 60 : 0;
			uint32_t trip_seconds = tripTime % 60;
			float tripDistance_mi = tripDistance_m / 1609.3;
			sprintf(buffer, "{\"tags\":{},\"values\":{\"id\":%lu,\"min\":%lu,\"sec\":%lu,\"mi\":%.1f}}", session_id, trip_minutes, trip_seconds, tripDistance_mi);
			publishQueue.publish("GNSS/session", buffer, 60, PRIVATE);
			break;
		}
		// 3 clicks: go into network standby mode, wake up very 22 or 120 minutes or when motion is detected.
		case 3: {
			// enable stop and reset session by turning off tracking.
			tracking_on_flag = false;
			reset_session_flag = true; // go for reset session id
			enter_standby_flag = true; // go to power save mode
			break;
		}
		// 4 clicks: enable turn off/on cellular module.
		case 4: {
			cycle_modem_power_flag = true;
			break;
		}
		default:
		break;
	}	
}


void autoSleepHandler() {
	// if no motion detected for AUTOSLEEP_TIME ms, go into sleep mode
	enter_standby_flag = true;
	#ifdef serial_debug
	Log.info("autosleep triggered.");
	#endif
}

// flash blue led if tracking is on
void trackingLEDUpdate() {
	if (state == TRACKING_ON_STATE) {
		digitalWrite(D7, HIGH);
		delay(50);
		digitalWrite(D7, LOW);
	}
}


#ifdef serial_debug
void cloud_statusHandler(system_event_t event, int data) {

	switch(data) {
		case cloud_status_connecting:
		Log.info("connecting to cloud");
		break;

		case cloud_status_connected:
		Log.info("connected to cloud");
		break;

		case cloud_status_disconnected:
		Log.info("disconnected from cloud");
		break;
	}
}
#endif


void resetGNSSInfo() {
	valid_fix_flag = false;
	lat = lon = alt = hAcc= speed_mph = 0.0;
	nb_sats = 0;
	// ttff = 0; // don't reset so we can use it to detect first time fix from reset from fix after sleep
	gnss_begin_ms = millis();
	gnss_last_fix_ms = millis();
}


void reset_publish_session() {
	// no session to publish, get new session id and exit
	if (tripTime == 0) {
		session_id_reset = true;
		return;
	}

	// convert seconds -> minutes:seconds and meters -> miles
	uint32_t trip_minutes = (tripTime > 60)? tripTime / 60 : 0;
	uint32_t trip_seconds = tripTime % 60;
	float tripDistance_mi = tripDistance_m / 1609.3;

	// format session stats
	const char *pattern = "{\"tags\":{},\"values\":{\"id\":%lu,\"min\":%lu,\"sec\":%lu,\"mi\":%.1f}}";
	snprintf(buffer, sizeof(buffer), pattern, session_id, trip_minutes, trip_seconds, tripDistance_mi);
	#ifdef serial_debug
	Serial.println(buffer);
	#endif
	// push to publish buffer so it'll be published next time it's cloud connected
	publishQueue.publish("GNSS/session", buffer, 60, PRIVATE);
	// reset trip data and set flag to renew session id
	tripTime = 0;
	tripDistance_m = 0.0;
	session_id_reset = true;
	return;
}


void status_bar_top_update() {
	display.drawFastHLine(0, 9, display.width(), WHITE);
	// cellular network cloud status indicator
	if(Particle.connected()) {
		//display.print("CC");
		display.drawBitmap(0, 0, cloud_icon, 10, 10, 1);
	} else {
		#ifdef cellular
		if(Cellular.connecting()) {
			display.drawBitmap(0, 0, signal_off_icon, 12, 10, 1);
		}
		if(Cellular.ready()) {
			display.drawBitmap(0, 0, signal_2G_icon, 12, 12, 1);
		}
		#else
		if(WiFi.connecting()) display.print("--");
		if(WiFi.ready()) display.print("WiFi");
		#endif
	}
	// gnss fix status indicator
	if (valid_fix_flag) {
		display.drawBitmap(14, 0, gnss_fix_icon_10, 10, 10, 1);
	} else {
		display.drawBitmap(14, 0, gnss_wait_icon_10, 10, 10, 1);
	}
	// tracking status indicator
	display.setCursor(28, 0);
	display.printf("%s", tracking_on_flag?"T":" ");
	// current local time
	display.setCursor(48, 0);
	display.printf("%d:%02d%s", Time.hourFormat12(), Time.minute(), Time.isAM()?"AM":"PM");
	// battery state of charge
	#ifdef cellular
	display.setCursor(104, 0);
	float soc = battery.getSoC();
	display.printf("%.0f%%", soc<100.0?soc:100.0);
	#endif
}


void status_bar_bottom_update() {
	//update bottom status bar
	display.drawFastHLine(0, 55, display.width(), WHITE);

	// left: trip time
	display.setCursor(0, 57);
	uint32_t trip_minutes = (tripTime > 60)? tripTime / 60 : 0;
	display.printf("%02d:%02d", trip_minutes, tripTime % 60);
	// center: last fix accuracy
	display.setCursor(49,57);
	if (valid_fix_flag) {
		display.printf("%.0fm", hAcc); // when valid,  accuracy in meters
	} else {
		display.printf("%d", nb_sats); // # sats while waiting for fix
		display.drawBitmap(55, 55, satellite_icon, 10, 10, 1);
	}
	// right: trip distance
	display.setCursor(79, 57);
	display.printf("%.2fmi", tripDistance_m/1609.3);
}


void display_update() {
	display.clearDisplay();
	status_bar_top_update();

	// time since gnss on and once a fix, delta lat/lon
	display.setCursor(0, 11);
	if (!valid_fix_flag) {
		if (Particle.connected()) {
			display.printf("%lus", (int)(millis() - gnss_last_fix_ms) / 1000);
		} else {
			if (imu_on) display.printf("T= %.0f F", imu.readTempF());
		}
	}

	// if lost cloud connection during tracking, display publish queue length
	if (!Particle.connected() && tracking_on_flag) {
		display.printf(" q:%u", publishQueue.getQueueSize());
	}

	/*
	char fixType_s[6];
	switch (fixType ) {
		case 0: {
			strcpy(fixType_s, "No fx"); //No fix
			break; }
		case 1: {
			strcpy(fixType_s, "DR"); // Dead Reckoning
			break; }
		case 2: {
			strcpy(fixType_s, "2D"); // 2d fix
			break; }
		case 3: {
			strcpy(fixType_s, "3D"); // 3d fix
			break; }
		case 4: {
			strcpy(fixType_s, "G+DR"); // GPS + dead reckoning
			break; }
		case 5: {
			strcpy(fixType_s, "T"); // Time only fix
			break; }
		default:
			break;
	}
	display.setCursor(47, 11);
	display.printf("%s", fixType_s);
	*/

	// display acceleration on a g-ball chart
	if (imu_on) {
		display.drawCircle(100, 32, 20, 1); // target circle
		display.drawFastHLine(100-20-3, 32, 2*20+2*3, 1); // target H line
		display.drawFastVLine(100, 32-20-2, 2*20+2*2, 1); // target V line
		// reset gball location to center
		int16_t gball_center_x = 100;
		int16_t gball_center_y = 32;

		// longitudinal accel along Z-axis
		// longitudinal acceleration displacement scaled for: 20 pixels/0.5G (auto-compensated for tilt )
		//if (accel[2] > 0.04 || accel[2] < -0.04) {
		gball_center_y = gball_center_y - int(accel[2]*40.0); //throttling --> accel < 0 --> y down --> ball up
		//}
		// lateral acceleration displacement scaled for: 20 pixels/0.5G along X-axis (not affected by tilt in this use case)
		if (accel[0] > 0.04 || accel[0] < -0.04 ) {
			gball_center_x = gball_center_x - int(accel[0]*40.0); //turn left --> accel > 0 --> x down --> ball left
		}

		// draw the gball
		display.fillCircle(gball_center_x, gball_center_y, 3, 1);

		// test: display acc_z offset 
		display.setCursor(0, 43);
		display.printf("z:%.2f", a0_z);
		// display.setCursor(41, 19);
		// display.printf("y:%.2f", a0_y);

		// debug data: acceleration vector
		
		// display.setCursor(0,43);
		// display.printf("m:%.3f", acc_mag);
		/*
		display.setCursor(48,28);
		display.print(accel[1]);
		display.setCursor(48,37);
		display.print(accel[2]);
		*/
	} else {
		display.setCursor(0, 19);
		display.printf("imu init. failed");
	}

	// display current speed (size 2)
	display.setCursor(11, 24);
	display.setTextSize(2);
	display.printf("%.0f", speed_mph);
	display.setTextSize(1);
	display.print("mph");

	// display current heading (size 2)
	display.setCursor(48, 35);
	display.setTextSize(2);
	display.print(valid_fix_flag?heading2cardinal(heading):"--");
	display.setTextSize(1);

	// snprintf # clicks when button is pressed...
	if (nb_clicks > 0) {
		display.setCursor(0, 43);
		display.printf("%lu clicks", nb_clicks);
		// clear the button pressed buffer
		nb_clicks = 0;
	}

	status_bar_bottom_update();
	display.display();
}


// functions included in Arduino but not Particle:
double radians(double deg) {
	return (deg * M_PI) / 180.0;
}
double degrees(double radians) {
	return (radians * 180.0) / M_PI;
}

// calculate distance or angle between two locations
// lat1 and long1 : previous locations
// lat2 and long2 : current locations
// approximation used: equirectangular (valid for short distances only)
// dist_angle: if True _> distance, else --> angle/bearing from old_location to current location
double distance_angle_between (float lat1, double long1, double lat2, double long2, bool dist_angle) {
	double dLon = radians(long2-long1);
	double aLat = radians(lat1+lat2)/2;
	double x = dLon*cos(aLat);
	double y = radians(lat2-lat1);
	if (dist_angle){
		// calculate distance
		double dist = sqrt(x*x + y*y);
		//Log.info("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f", dLon, aLat, x, y, dist);
		//return distance in meters after multiplying by mean earth radius
		return dist*6371009.0;
	}
	else {
		// returns angle in degrees!
		// to get bearing relative to course: rel_bearing = bearingTo - course
		// it's in [-pi, pi] and positive -> turn right, negative-> turn left
		double cto= atan2(x, y);
		//cto < 0.0? cto += TWO_PI: cto;
		//or: (degress(cto)+360)%360 to have it in 0..359
		return degrees(cto);
	}
}

// from Mikal Hart's TinyGPSPlus lib: convert heading in degrees into a cardinal direction: N, NW, ENE ....
const char* heading2cardinal(double heading) {
	static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
	int direction = (int)((heading + 11.25) / 22.5);
	return directions[direction % 16];
}

bool init_imu() {
	//Over-ride default settings if desired
	imu.settings.gyroEnabled = 0;  //to be reset as there's no need for gyro in this project for now
	imu.settings.gyroRange = 1000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
	imu.settings.gyroSampleRate = 13;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
	imu.settings.gyroBandWidth = 50;  //Hz.  Can be: 50, 100, 200, 400;
	imu.settings.gyroFifoEnabled = 0;  //Set to include gyro in FIFO
	imu.settings.gyroFifoDecimation = 1;  //set 1 for on /1

	imu.settings.accelEnabled = 1;
	imu.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16
	imu.settings.accelSampleRate = 52;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
	imu.settings.accelBandWidth = 100;  //Hz.  Can be: 50, 100, 200, 400;
	imu.settings.accelFifoEnabled = 0;  //Set to include accelerometer in the FIFO
	imu.settings.accelFifoDecimation = 2;  //set 1 for on /1

	//Non-basic mode settings
	imu.settings.commMode = 1; // doesn't seem to do anything, no reference to AN either...

	//FIFO control settings
	//imu.settings.fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes)
	//imu.settings.fifoSampleRate = 10;  //Hz.  Can be: 10, 25, 50, 100, 200, 400, 800, 1600, 3300, 6600
	imu.settings.fifoModeWord = 0;  //FIFO mode.
	//FIFO mode.  Can be:
	//  0 (Bypass mode, FIFO off)
	//  1 (Stop when full)
	//  3 (Continuous during trigger)
	//  4 (Bypass until trigger)
	//  6 (Continous mode)

	imu.settings.tempEnabled = 1; // enable temp. data

	status_t imu_init_flag = imu.begin();

	return imu_init_flag == IMU_SUCCESS? true : false;
}

bool enable_WakeOnMotionInterrupt() {
	// configure INT1 output to trigger wake on motion
	uint8_t errorAccumulator = 0;
	uint8_t dataToWrite = 0;
	// ST's recommended  implementation (AN4650 p.50)
	// 1- set ODR (sampling rate) and full scale settings in CTRL1_XL
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
	errorAccumulator = imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
	// 2- write 0x10 to TAP_CFG: enable high pass filter, no latch on INT (INT is high for just 1/ODR sec)
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x10 );
	// 3- write 0x00 to WAKE_UP_DUR (bits5&6) in multiple of 1/ODR second
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x40 ); //for 2 samples long
	// 4- write 0x04 to WAKE_UP_THS (0h to 3Fh) to set detection threshold in multiples of FS/64
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x05 ); //0x04:0.125g, 0x05:0.156, 0x08:0.25g
	// 5- write 0x20 to MD1_CFG to drive wake up interrupt to INT1
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x20 );
	return errorAccumulator == 0? true:false;
}

bool disable_WakeOnMotionInterrupt() {
	uint8_t errorAccumulator = 0;
	errorAccumulator = imu.writeRegister( LSM6DS3_ACC_GYRO_MD1_CFG, 0x00 );
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_TAP_CFG1, 0x00 );
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_DUR, 0x00 );
	errorAccumulator += imu.writeRegister( LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x00 );
	return errorAccumulator == 0? true:false;
}

// checking with wtachdog every 5 sec if cloud connected 
void wd_check_in() {
	if (Particle.connected()) {
		wd.checkin();
	}
}