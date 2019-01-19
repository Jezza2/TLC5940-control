/*
 * TLC5940_control.c
 * 
 * Copyright 2015 Jeremy Stanger <jeremy@Jeremy-PC>
 * 
 * You are free to edit this code in any way you like and
 * redistribute it as you wish.
 * 
 */
 
/*
 * This code is known to run on an Arduino Uno or Nano.
 * 
 * It controls the TLC5940, a constant current sink PWM driver, and
 * is designed specifically to work with RGB LEDs.
 * 
 * It was written because some of the features of the already available
 * TLC5940 Arduino library no longer work due to updates to the Arduino
 * environment.  Also it was an interesting and enjoyable academic
 * exercise.
 * 
 * Included at the end are visual effects I programmed.  These might be
 * useful to you, but are only included because they're the ones I
 * used myself.
 */

#define GSCLK_PORT		PORTD	// Grayscale clock
#define VPRG_PORT		PORTD	// Grayscale/Dot correction mode
#define XLAT_PORT		PORTB	// Latch grayscale/dot correction data
#define BLANK_PORT		PORTB	// Switch off all outputs and reset grayscale counter
#define SIN_PORT		PORTB	// Write grayscale/dot correction data
#define SCLK_PORT		PORTB	// Clock each bit of grayscale/dot correction data
#define HDSHK_PORT		PORTD	// Send handshake signal to confirm reception of advance/go back a cue command
#define RCV_ADV_IN		PIND	// Receive advance a cue
#define RCV_BAK_IN		PIND	// Receive go back a cue

// Pin mapping relative to port
#define GSCLK			3		// Pin 3
#define VPRG			4		// Pin 4
#define XLAT			1		// Pin 9
#define BLANK			2		// Pin 10
#define SIN				3		// Pin 11
#define SCLK			5		// Pin 13
#define RCV_ADV			5		// Pin 5
#define HDSHK			6		// Pin 6
#define RCV_BAK			7		// Pin 7

// The order of the tricolour legs (L2R) to ensure the right colour comes on!
#define RED_L			2
#define GREEN_L			1
#define BLUE_L			0

// Controls the current through the LEDs of each colour
#define RED_CURRENT		27
#define GREEN_CURRENT	16
#define BLUE_CURRENT	21

// PWM frequency is given by f_0/(4096*GSCLK_PERIOD)
// f_0 is the frequency of the system clock = 16MHz
// Period of GSCLK in system clock cycles
#define GSCLK_PERIOD	1
// The brightness is actually inversely proportional to this constant.
// Minimum value 1, maximum value 16/GSCLK_PERIOD.
// Period of BLANK clock in units of grayscale cycles
#define LED_BRIGHTNESS	1

// Number of chips and LEDs to control
#define NUM_TLC			2
#define NUM_LED			9

// Design #defines to assist FX programming
// Colours
#define BLACK			0,0,0
#define WHITE           255,255,255
#define RED             255,0,0
#define GREEN           0,255,0
#define BLUE            0,0,255
#define YELLOW          255,255,0
#define PINK            255,0,255
#define CYAN            0,255,255
#define ORANGE          255,128,0
#define PURPLE          186,85,211
#define GOLD            255,150,37

// Lookup table to account for the non-linear realationship between
// absolute brightness and perceived brightness.
// Reduces PWM to 8 bit, but fading is smoother.
const unsigned int PWM_VALUE[] = {0,1,3,5,7,9,11,12,14,20,21,22,23,25,26,27,29,
  31,32,34,36,37,39,41,43,45,47,49,52,54,56,59,61,64,66,69,72,75,77,80,83,87,90,
  93,96,100,103,107,111,115,118,122,126,131,135,139,144,148,153,157,162,167,172,
  177,182,187,193,198,204,209,215,221,227,233,239,246,252,259,265,272,279,286,293,
  300,308,315,323,330,338,346,354,362,371,379,388,396,405,414,423,432,442,451,461,
  470,480,490,501,511,521,532,543,553,564,576,587,598,610,622,634,646,658,670,683,
  695,708,721,734,748,761,775,788,802,816,831,845,860,874,889,904,920,935,951,966,
  982,999,1015,1031,1048,1065,1082,1099,1116,1134,1152,1170,1188,1206,1224,1243,1262,
  1281,1300,1320,1339,1359,1379,1399,1420,1440,1461,1482,1503,1525,1546,1568,1590,
  1612,1635,1657,1680,1703,1726,1750,1774,1797,1822,1846,1870,1895,1920,1945,1971,
  1996,2022,2048,2074,2101,2128,2155,2182,2209,2237,2265,2293,2321,2350,2378,2407,
  2437,2466,2496,2526,2556,2587,2617,2648,2679,2711,2743,2774,2807,2839,2872,2905,
  2938,2971,3005,3039,3073,3107,3142,3177,3212,3248,3283,3319,3356,3392,3429,3466,
  3503,3541,3578,3617,3655,3694,3732,3772,3811,3851,3891,3931,3972,4012,4054,4095};

// Index variable
unsigned int loop_var = 0;
// Flag indicating whether there is data in the serial register
// waiting to be latched into the grayscale register
byte data_waiting = 0;

// grayscale_values holds the current values in the grayscale register
byte grayscale_values[16*NUM_TLC];

// ========= SETUP FUNCTIONS ===========================================

// stands for Interrupt Service Routine
// This is called at the end of every grayscale cycle by a hardware level interrupt
ISR(TIMER1_COMPA_vect) {
  reset_counter();
  // Set value in timer register to 0 to avoid BLANK and GSCLK getting out of sync
  TCNT1 = 0;
}

// Initialises the timers used for BLANK and GSCLK
void init_timers() {
  
  /*GSCLK - Timer 2*/
  
  // Disable interrupts while setting up clock
  cli();
  
  // Clock settings, using timer 2
  // This should give a square wave, duty cycle 50%, frequency 16MHz
  // Achieved is actually ~14MHz, but siginificant uncertainty in this measurement.
  // Turn on pin B (3) | CTC mode   | fast PWM
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  
  // OCR2B controlling top limit | set prescaler to 1, i.e. no prescaling
  TCCR2B = _BV(WGM22) | _BV(CS20);
  // Compare registers.  Counter resets and pin goes HIGH on reaching value in A, then goes LOW on reaching value in B.
  OCR2A = GSCLK_PERIOD;
  OCR2B = 0;
  

  /*BLANK - Timer1 */
  // Clear OC1B on compare|fast pwm | fast pwm
  TCCR1A = _BV(COM1B0) |  _BV(WGM10) | _BV(WGM11);
  //         fast pwm | fast pwm   |  No prescaling
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS10);
  TCNT1 = 0;
  // Compare register set to give period of 4096 clock oscillations (3209.6 Hz)
  OCR1A = 4096 * GSCLK_PERIOD * LED_BRIGHTNESS - 1;
  // enable timer compare interrupt:
  TIMSK1 |= _BV(OCIE1A);
  
  
  // Enable global interrupts
  sei();
}

// Writes dot correction data to TLC for the green, red and blue LEDs
void write_dc_data(byte red_val, byte green_val, byte blue_val) {
  // VPRG high for dot correction programming mode
  VPRG_PORT |= _BV(VPRG);
  
  for (loop_var = 0; loop_var < NUM_TLC * 16 * 6; loop_var ++) {	
	// SCLK low and prepare SIN for data
	// NB: SCLK_PORT == SIN_PORT
	SCLK_PORT &= ~(_BV(SCLK) | _BV(SIN));
	// Send next byte of data
	if ((NUM_TLC*16 - loop_var/6 - 1)%3 == RED_L) {
	  if (red_val & (32 >> (loop_var%6))) {
	    SIN_PORT |= _BV(SIN);
	  }
    } else if ((NUM_TLC*16 - loop_var/6 - 1)%3 == GREEN_L) {
	  if (green_val & (32 >> (loop_var%6))) {
	    SIN_PORT |= _BV(SIN);
	  }
    } else {
	  if (blue_val & (32 >> (loop_var%6))) {
	    SIN_PORT |= _BV(SIN);
	  }
    }
    // SCLK high - clock the bit into the input register
    SCLK_PORT |= _BV(SCLK);
  }
  // Leave SCLK low
  SCLK_PORT &= ~_BV(SCLK);
	
  // Latch data into the DC registers
  XLAT_PORT |= _BV(XLAT);
  XLAT_PORT &= ~_BV(XLAT);

  // VPRG low for grayscale programming mode
  VPRG_PORT &= ~_BV(VPRG);
  
  // Sends an extra clock pulse.
  // Data sheet says this needs to happen to complete the grayscale
  // cycle whenever DC values have been written.  Not sure why.
  SCLK_PORT |= _BV(SCLK);
  SCLK_PORT &= ~_BV(SCLK);
}

void setup() {
  // Assign pin modes:  (Leaving pin 3 & 10 off since I don't want the 
  // grayscale and blank clocks to start yet)
  DDRD |= _BV(VPRG) | _BV(HDSHK) & ~_BV(RCV_ADV) & ~_BV(RCV_BAK);
  DDRB |= _BV(XLAT) | _BV(SIN) | _BV(SCLK);
  
  // Set outputs to initial desired states (i.e. all low)
  PORTD &= B00000000;
  PORTB &= B11000000;

  // Initialise timers
  init_timers();
  
  // Write dot correction data for red, green and blue (respectively)
  write_dc_data(RED_CURRENT, GREEN_CURRENT, BLUE_CURRENT);
  // Send grayscale data to TLC
  write_gs_data();
  // Enable grayscale clock and blank
  toggle_gsclk();
  toggle_blank();
  // Start grayscale cycle
  reset_counter();
  
  channel_set_all(0);
  write_gs_data();
  
  
  led_set_all(0, 0, 0, 1);
}

// ========= HARDWARE INTERFACE FUNCTIONS ==============================

// Sends grayscale data to the TLCs
void write_gs_data() {
  for (loop_var = 0; loop_var < NUM_TLC * 16 * 12; loop_var ++) {	
	// SCLK low and prepare SIN for data
	// NB: SCLK_PORT == SIN_PORT
	SCLK_PORT &= ~(_BV(SCLK) | _BV(SIN));
	// Send next byte of data
	// 2048 because it is 100000000000 and so shifts MSB first
	// 1 would be used if LSB first was required
	if (PWM_VALUE[grayscale_values[NUM_TLC*16 - loop_var / 12 - 1]] & (2048 >> (loop_var%12))) {
      SIN_PORT |= _BV(SIN);
    }
    // SCLK high - clock bit into input register
    SCLK_PORT |= _BV(SCLK);
  }
  // Leave SCLK low
  SCLK_PORT &= ~_BV(SCLK);
    
  // XLAT may only be pulled at the end of a grayscale cycle, so instead,
  // set a variable saying the data is waiting to be latched.  Then
  // latch during the reset_counter method.
  data_waiting = 1; 
}

// Executed at the end of every grayscale cycle, resets grayscale counter
void reset_counter() {
  // BLANK HIGH - switch all outputs off and reset the grayscale counter
  BLANK_PORT |= _BV(BLANK);
  
  // Disable grayscale clock
  toggle_gsclk();
  
  /*latch new data if waiting*/
  if (data_waiting) {
	// XLAT high
	XLAT_PORT |= _BV(XLAT);
	// Data no longer waiting
	data_waiting = 0;
	// XLAT low
	XLAT_PORT &= ~_BV(XLAT);
  }
  
  // Enable grayscale clock again
  toggle_gsclk();
  
  // BLANK low - enable outputs and begin new grayscale cycle
  BLANK_PORT &= ~_BV(BLANK);
}

// Switches the grayscale clock on or off by disabling/enabling pin 3 as an output
void toggle_gsclk() {
  DDRD ^= _BV(GSCLK);
}

// Switches the blank signal on or off by disabling/enabling pin 10 as an output
void toggle_blank() {
  DDRB ^= _BV(BLANK);
}

// Set a specific channel with a brightness given by val.
// val is an 8 bit integer (0 - 255) and is converted into a 12 bit one
// before being sent to the TLC
void channel_set(byte channel, byte val) {
  grayscale_values[channel] = val;
}

// Set all channels to the same brightness given by val
void channel_set_all(byte val) {
  if (val < 4096) {
	for (loop_var = 0; loop_var < 16 * NUM_TLC; loop_var++) {
	  channel_set(loop_var, val);
	}
  }
}

// ========= PROGRAMMING FUNCTIONS =====================================

// fade_speeds holds the speed at which grayscale values approaches new grayscale values.
// a fade speed of 0 indicates no fade, just an instant switch.
byte fade_speeds[NUM_LED];
// new_grayscale_values holds the values the current grayscale values are fading towards
byte new_grayscale_values[16*NUM_TLC];
byte index = 0;
byte fade_counter = 0;

// The cue number - changed only when a cue advance command is received
int cue = 0;
// These two variables can be used to advance the cue number automatically
// after a preset amount of time.
byte sub_cue = 0;
long auto_advance_counter = 0;

/*
 * This array takes some explaining.
 * At all times, there are 4 colours defined:
 * Background (BG), Foreground1 (FG1), Foreground2 (FG2) and ForegroundCurrent (FGC).
 * These are used by the effect functions in various ways, but the background colour
 * is what things fade back to; kind of like a virtual black.
 * i.e. when animation functions refer to 'off' they mean 'background colour,'
 * except the function 'all_off' which sets all channels to 0.
 * The two foreground colours are then what the function switches between for the actual effect.
 * Foreground current is the current colour, somewhere between the two foreground
 * colours
 * 
 * The way in which it switches between foreground colours is determined by fade_style.
 * See assign_colours function for specifics.
 * 
 * You can also set the number of increments; i.e. how many different shades
 * there are between the 2 foreground colours.
 * 
 * DIR is used if the fade style is smooth (as opposed to random).
 * If DIR is 1 then the colour is changing from FG1 to FG2, and vice
 * versa if DIR is -1.
 * 
 * The constants just refer locations in the colours array and should not
 * be changed.  They are used for readability.
 * 
 * There is an RGB for each colour and an increment for each of RGB as well.
 * 
 * The overall point of this array is facilitate effects being of more than
 * one colour; instead they can be any colour along a spectrum.
 */
byte colours[18];

#define BG_RED			0
#define BG_GREEN		1
#define	BG_BLUE			2
#define FG1_RED			3
#define FG1_GREEN		4
#define	FG1_BLUE		5
#define FG2_RED			6
#define FG2_GREEN		7
#define	FG2_BLUE		8
#define FGC_RED			9
#define FGC_GREEN		10
#define	FGC_BLUE		11
#define FADE_STYLE		12
#define NUM_INC			13
#define INC_RED			14
#define INC_GREEN		15
#define INC_BLUE		16
#define DIR				17

unsigned int anim_count = 0;
byte off_speed = 0;

// Sets the led colour directly, no fading
void led_set(byte led, byte R, byte G, byte B) {
  channel_set(3*led + RED_L, R);
  channel_set(3*led + GREEN_L, G);
  channel_set(3*led + BLUE_L, B);
}

// Sets the new state for an led so it
void led_set_new(byte led, byte R, byte G, byte B, byte fade) {
  new_grayscale_values[3*led + RED_L] = R;
  new_grayscale_values[3*led + GREEN_L] = G;
  new_grayscale_values[3*led + BLUE_L] = B;
  fade_speeds[led] = fade;
}

void led_set_all(byte R_A, byte G_A, byte B_A, byte fade_a) {
  for (index = 0; index < NUM_LED; index++) {
	led_set_new(index, R_A, G_A, B_A, fade_a);
  }
}

// Collection of functions for getting the current and,
// if applicable, the future state of LEDs.
byte get_led_red(byte led) {
  return grayscale_values[3*led + RED_L];
}

byte get_led_blue(byte led) {
  return grayscale_values[3*led + BLUE_L];
}

byte get_led_green(byte led) {
  return grayscale_values[3*led + GREEN_L];
}

byte get_new_led_red(byte led) {
  return new_grayscale_values[3*led + RED_L];
}

byte get_new_led_blue(byte led) {
  return new_grayscale_values[3*led + BLUE_L];
}

byte get_new_led_green(byte led) {
  return new_grayscale_values[3*led + GREEN_L];
}

// Possibly the messiest function I've ever written.
// This loops through all LEDs.  If the grayscale value is not equal
// to the new grayscale value, then the grayscale value will be changed
// by a maximum of the fade value.  If the difference between the
// grayscale value and the new grayscale value is less than the fade
// value, then the grayscale value is set equal to the new grayscale value.
// Special case:  If the fade value == 0, then the grayscale value is
// set immediately to the new grayscale value.
void perform_fades() {
  
  byte new_red = 0;
  byte new_green = 0;
  byte new_blue = 0;
  
  for (index = 0; index < NUM_LED; index++) {
	  
	new_red = grayscale_values[3*index + RED_L];
	new_green = grayscale_values[3*index + GREEN_L];
	new_blue = grayscale_values[3*index + BLUE_L];
	
	if (fade_speeds[index] == 0) {
		
	  new_red = new_grayscale_values[3*index + RED_L];
	  new_green = new_grayscale_values[3*index + GREEN_L];
	  new_blue = new_grayscale_values[3*index + BLUE_L];
	  
	} else if (grayscale_values[3*index + RED_L] != new_grayscale_values[3*index + RED_L]
	          || grayscale_values[3*index + GREEN_L] != new_grayscale_values[3*index + GREEN_L]
	          || grayscale_values[3*index + BLUE_L] != new_grayscale_values[3*index + BLUE_L]) {
	  
	  if (grayscale_values[3*index + RED_L] < new_grayscale_values[3*index + RED_L]) {
		if (new_grayscale_values[3*index + RED_L] - grayscale_values[3*index + RED_L] < fade_speeds[index]) {
		  new_red = new_grayscale_values[3*index + RED_L]; 
		} else {
		  new_red = grayscale_values[3*index + RED_L] + fade_speeds[index];
		}
	  } else if (grayscale_values[3*index + RED_L] > new_grayscale_values[3 * index + RED_L]) {
		if (grayscale_values[3*index + RED_L] - new_grayscale_values[3*index + RED_L] < fade_speeds[index]) {
		  new_red = new_grayscale_values[3*index + RED_L];
		} else {
		  new_red = grayscale_values[3*index + RED_L] - fade_speeds[index];
		}
	  }
	  
	  if (grayscale_values[3*index + GREEN_L] < new_grayscale_values[3*index + GREEN_L]) {
		if (new_grayscale_values[3*index + GREEN_L] - grayscale_values[3*index + GREEN_L] < fade_speeds[index]) {
		  new_green = new_grayscale_values[3*index + GREEN_L]; 
		} else {
		  new_green = grayscale_values[3*index + GREEN_L] + fade_speeds[index];
		}
	  } else if (grayscale_values[3*index + GREEN_L] > new_grayscale_values[3 * index + GREEN_L]) {
		if (grayscale_values[3*index + GREEN_L] - new_grayscale_values[3*index + GREEN_L] < fade_speeds[index]) {
		  new_green = new_grayscale_values[3*index + GREEN_L];
		} else {
		  new_green = grayscale_values[3*index + GREEN_L] - fade_speeds[index];
		}
	  }
	  
	  if (grayscale_values[3*index + BLUE_L] < new_grayscale_values[3*index + BLUE_L]) {
		if (new_grayscale_values[3*index + BLUE_L] - grayscale_values[3*index + BLUE_L] < fade_speeds[index]) {
		  new_blue = new_grayscale_values[3*index + BLUE_L]; 
		} else {
		  new_blue = grayscale_values[3*index + BLUE_L] + fade_speeds[index];
		}
	  } else if (grayscale_values[3*index + BLUE_L] > new_grayscale_values[3 * index + BLUE_L]) {
		if (grayscale_values[3*index + BLUE_L] - new_grayscale_values[3*index + BLUE_L] < fade_speeds[index]) {
		  new_blue = new_grayscale_values[3*index + BLUE_L];
		} else {
		  new_blue = grayscale_values[3*index + BLUE_L] - fade_speeds[index];
		}
	  }
	}
	
	led_set(index, new_red, new_green, new_blue);	
  }
  fade_counter++;
}

void loop() {

  //  Advance cue number if signal is recieved on pin 5
  //  Go back a cue if a signal is recieved on pin 7
  //  The HDSHK pin is pulsed to tell the other arduino that the signal has been recieved.
  if (RCV_ADV_IN & _BV(RCV_ADV)) {
    HDSHK_PORT |= _BV(HDSHK);
    cue++;
    auto_advance_counter = 0;
    anim_count = 0;
    sub_cue = 0;
  }
  if (RCV_BAK_IN & _BV(RCV_BAK)) {
    HDSHK_PORT |= _BV(HDSHK);
    if (cue > 0) {
	  cue--;
	  auto_advance_counter = 0;
	  anim_count = 0;
          sub_cue = 0;
	}
  }
  
  animate();
  perform_fades();
  write_gs_data();
  delayMicroseconds(200);
  
  HDSHK_PORT &= ~_BV(HDSHK);
}

// This function is where the animation functions are called from
// I've left my animations as examples of how you might programme a list
// of cues.
void animate() {

  if (cue == 1) {
        
    if (auto_advance_counter == 250 || auto_advance_counter == 500 || auto_advance_counter == 750) {
      anim_count = 0;
      sub_cue++;
    }
    
	if (sub_cue == 0) {
      assign_colours(BLACK, RED, RED, 0, 1);
      all_on(0, 0);
	} else if (sub_cue == 1) {
	  assign_colours(BLACK, GREEN, GREEN, 0, 1);
	  all_on(0, 0);
	} else if (sub_cue == 2) {
	  assign_colours(BLACK, BLUE, BLUE, 0, 0);
	  all_on(0, 0);
	} else {
          sub_cue = 0;
	  auto_advance_counter = 0;
	}
	
  } else if (cue == 2) {
	all_off();
	
  } else if (cue == 3) {
	// Whenever one of these conditions is true, the subcue number automatically
	// advances.  Auto_advance_counter is incremented in the animation functions
	// at various rates which depend on the effect.
    if (auto_advance_counter == 2000 || auto_advance_counter == 3000 
    || auto_advance_counter == 4000 || auto_advance_counter == 5000) {
          
      anim_count = 0;
      sub_cue++;
    }
    
	if (sub_cue == 0) {
	  // For each sub cue, first, colours are assigned
	  // The background colour, the two foreground colours, the fade_style
	  // and then the number of shades in between the two colours that are
	  // available
 	  assign_colours(BLACK, BLUE, WHITE, 1, 255);
 	  // Call the effect
	  raindrops(1, 1, 15, 5, 0);
	} else if (sub_cue == 1) {
	  assign_colours(BLACK, BLUE, WHITE, 4, 255);
	  pattern_shift(448, 9, 0, 0, 1, 1);
	} else if (sub_cue == 2) {
	  assign_colours(BLACK, BLUE, WHITE, 3, 255);
	  pattern_invert(341, 30, 0, 0);
	} else if (sub_cue == 3) {
	  assign_colours(BLACK, BLUE, WHITE, 3, 255);
	  runners(7, 1, 25, 8, 0, 0);
	} else {
	  sub_cue = 1;
	  auto_advance_counter = 2001;
	}
	
  } else if (cue == 4) {
	all_off();
	
  } else if (cue == 5) {
	if (auto_advance_counter == 1560 || auto_advance_counter == 1660 
        || auto_advance_counter ==  2300 || auto_advance_counter == 2400
        || auto_advance_counter == 2500 || auto_advance_counter == 2600
        || auto_advance_counter == 3100 || auto_advance_counter == 4620) {
          
          anim_count = 0;
          sub_cue++;
        }
    
	if (sub_cue == 0) {
	  assign_colours(0, 50, 0, 0, 0, 50, GREEN, 0, 255);
      fades(29, 3, 3);
	} else if (sub_cue == 1) {
	  assign_colours(BLUE, BLUE, GREEN, 0, 255);
      all_on(3, 3);
	} else if (sub_cue == 2) {
	  assign_colours(0, 50, 0, 0, 0, 50, GREEN, 0, 255);
      fades(29, 3, 3);
	} else if (sub_cue == 3) {
	  assign_colours(BLUE, GREEN, GREEN, 0, 255);
      all_on(3, 3);
	} else if (sub_cue == 4) {
	  assign_colours(0, 50, 0, 0, 0, 50, GREEN, 0, 255);
      fades(29, 3, 3);
	} else if (sub_cue == 5) {
	  assign_colours(BLUE, BLUE, GREEN, 0, 255);
      all_on(3, 3);
	} else if (sub_cue == 6) {
	  assign_colours(0, 50, 0, 0, 0, 50, GREEN, 0, 255);
      fades(29, 3, 3);
	} else if (sub_cue == 7) {
	  assign_colours(BLACK, BLUE, GREEN, 1, 255);
      fades(32, 0, 0);
	} else if (sub_cue == 8) {
	  assign_colours(BLACK, BLUE, GREEN, 2, 255);
      runners(3, 1, 0, 30, 0, 1);
	} else {
      sub_cue = 0;
	  auto_advance_counter = 0;
	}
	
  } else if (cue == 6) {
    all_off();
    
  } else if (cue == 7) {
	assign_colours(BLACK, BLUE, PINK, 1, 255);
	raindrops(15, 2, 5, 5, 1);
	
  } else if (cue == 8) {
	all_off();
	
  } else if (cue == 9) {
	if (auto_advance_counter == 220 || auto_advance_counter == 240 
        || auto_advance_counter ==  430 || auto_advance_counter == 450
        || auto_advance_counter == 470 || auto_advance_counter == 490
        || auto_advance_counter == 650 || auto_advance_counter == 670
        || auto_advance_counter ==  860 || auto_advance_counter == 880
        || auto_advance_counter == 900 || auto_advance_counter == 920
        || auto_advance_counter == 1080 || auto_advance_counter == 1100
        || auto_advance_counter ==  1290 || auto_advance_counter == 1310
        || auto_advance_counter == 1330 || auto_advance_counter == 1350
        || auto_advance_counter == 1510 || auto_advance_counter == 1530
        || auto_advance_counter ==  1720 || auto_advance_counter == 1740
        || auto_advance_counter == 1760 || auto_advance_counter == 1780
        || auto_advance_counter == 1940) {
          
          anim_count = 0;
          sub_cue++;
        }
        
        assign_colours(BLACK, RED, PINK, 1, 255);
    
	if (sub_cue == 0) {
      all_off();
      auto_advance_counter++;
	} else if (sub_cue == 1) {
      all_on(0, 5);
	} else if (sub_cue == 2) {
      fades(45, 7, 7);
	} else if (sub_cue == 3) {
      all_on(0, 0);
	} else if (sub_cue == 4) {
      all_off();
      auto_advance_counter++;
	} else if (sub_cue == 5) {
      all_on(0, 2); 
    } else if (sub_cue == 6) {
      all_off();
      auto_advance_counter++;
          
	} else if (sub_cue == 7) {
      all_on(0, 5);
	} else if (sub_cue == 8) {
      fades(45, 7, 7);
	} else if (sub_cue == 9) {
      all_on(0, 0);
	} else if (sub_cue == 10) {
      all_off();
      auto_advance_counter++;
	} else if (sub_cue == 11) {
      all_on(0, 2); 
    } else if (sub_cue == 12) {
      all_off();
      auto_advance_counter++;
      
	} else if (sub_cue == 13) {
      all_on(0, 5);
	} else if (sub_cue == 14) {
      fades(45, 7, 7);
	} else if (sub_cue == 15) {
      all_on(0, 0);
	} else if (sub_cue == 16) {
      all_off();
      auto_advance_counter++;
	} else if (sub_cue == 17) {
      all_on(0, 2); 
    } else if (sub_cue == 18) {
      all_off();
      auto_advance_counter++;
    
    } else if (sub_cue == 19) {
      all_on(0, 5);
	} else if (sub_cue == 20) {
      fades(45, 7, 7);
	} else if (sub_cue == 21) {
      all_on(0, 0);
	} else if (sub_cue == 22) {
      all_off();
      auto_advance_counter++;
	} else if (sub_cue == 23) {
      all_on(0, 2); 
    } else if (sub_cue == 24) {
      all_off();
      auto_advance_counter++;
          
	} else {
          raindrops(1, 1, 20, 20, 0);
	}
  }
  else if (cue == 10) {
    all_off();
  }
  
  else if (cue == 11) {
    if (auto_advance_counter == 450 || auto_advance_counter == 2850) {
        
      anim_count = 0;
      sub_cue++;
    }
        
    assign_colours(BLACK, RED, ORANGE, 3, 50);
     
    if (sub_cue == 0) {
      runners(7, 1, 0, 0, 0, 1);
    } else if (sub_cue == 1) {
      runners(13, 1, 7, 7, 0, 1);
    } else {
      pattern_invert(455, 23, 0, 0);
    }
  }
  
  else if (cue == 12) {
    all_off();
  }
  
  else if (cue == 13) {
        
    if (auto_advance_counter == 5 || auto_advance_counter == 3625) {
      anim_count = 0;
      sub_cue++;
    }
        
    assign_colours(BLACK, RED, ORANGE, 2, 10);
        
    if (sub_cue == 0) {
      all_off();
      auto_advance_counter++;
    } else if (sub_cue == 1) {
      fades(113, 0, 4);
    } else if (sub_cue == 2) {
      pattern_shift(301, 10, 0, 0, 1, 0);
    }
  }
  
  else if (cue == 14) {
    all_off();
  }
  
  else if (cue == 15) {
        
    if (auto_advance_counter == 5 || auto_advance_counter == 3625) {
      anim_count = 0;
      sub_cue++;
    }
        
    if (sub_cue == 0) {
      all_off();
      auto_advance_counter++;
    } else if (sub_cue == 1) {
      assign_colours(ORANGE, RED,YELLOW, 2, 10);
      fades(113, 0, 4);
    } else if (sub_cue == 2) {
      assign_colours(ORANGE, BLACK, BLACK, 0, 1);
      raindrops(10, 1, 20, 5, 0);
    }
  }
  
  else if (cue == 16) {
    all_off();
  }
  
  else if (cue == 17) {
    if (auto_advance_counter == 5 || auto_advance_counter == 115) {
      anim_count = 0;
      sub_cue++;
    }
        
    assign_colours(BLACK, BLACK, WHITE, 3, 20);
        
    if (sub_cue == 0) {
      auto_advance_counter++;
    } else if (sub_cue == 1) {
      fades(2, 0, 0);
    } else if (sub_cue == 2) {
      all_off();
    }
  }
  
  else if (cue == 18) {
    if (auto_advance_counter == 1400 || auto_advance_counter == 5000) {
      anim_count = 0;
      sub_cue++;
    }
        
    assign_colours(BLACK, RED, ORANGE, 3, 20);
        
    if (sub_cue == 0) {
      runners(8, 1, 9, 9, 0, 0);
    } else if (sub_cue == 1) {
      counting(100, -1, 10, 2, 1, 1, 0, 0, 0);
    } else if (sub_cue == 2) {
      all_off();
    }
  }
  
  else if (cue == 19) {
    all_off();
  }
  
  else if (cue == 20) {
    assign_colours(0, 0, 80, BLACK, WHITE, 1, 255);
    raindrops(50, 1, 6, 3, 0);
  }
  
  else if (cue == 21) {
    all_off();
  }
  
  else if (cue == 22) {
    assign_colours(0, 0, 80, WHITE, WHITE, 0, 255);
    runners(35, 1, 5, 1, 1, 0);
  }
  
  else if (cue == 23) {
    all_off();
  }
  
  else if (cue == 24) {
    if (auto_advance_counter == 685) {
      anim_count = 0;
      sub_cue++;
    }
        
    assign_colours(BLACK, RED, ORANGE, 2, 20);
        
    if (sub_cue == 0) {
      fades(118, 0, 2);
    } else {
      counting(237, -1, 0, 0, 1, 0, 0, 0, 0);
    }
  }

  else if (cue == 25) {
    all_off();
  }
  
  else if (cue == 26) {
	if (auto_advance_counter == 1000) {
      anim_count = 0;
      auto_advance_counter = 0;
      sub_cue++;
    }
        
    if (sub_cue == 0) {
      assign_colours(BLACK, RED, GREEN, 3, 50);
    } else if (sub_cue == 1) {
      assign_colours(BLACK, GREEN, BLUE, 3, 50);
    } else if (sub_cue == 2) {
      assign_colours(BLACK, BLUE, RED, 3, 50);
    } else {
	  sub_cue = 0;
	  auto_advance_counter = 0;
	}
    
    raindrops(20, 2, 20, 10, 0);
  }
//  } else if (cue == 50) {
//	assign_colours(BLACK, RED, PINK, 3, 255);
//	runners(10, 1, 7, 7, 0, 1);
//  } else {
//    all_off();
//  }
}

void all_off() {
  led_set_all(0, 0, 0, off_speed);
}

// Checks whether or not the given LED has finished fading to its 'destination' colour.
// A 1 means the LED is not fading.
byte test_not_fading(byte led) {
  if (get_led_red(led) == get_new_led_red(led) 
    && get_led_green(led) == get_new_led_green(led) 
    && get_led_blue(led) == get_new_led_blue(led)) {
	
	return 1;
  } else {
	return 0;
  }
}

// Assign the colours for the background to fade between
// and the colours for the foreground to fade between.
// fade_style takes the value 0, 1, 2, 3 or 4.
// 0 means don't change.
// 1 means randomise the colours along the spectrum; change with time
// 2 means randomise the colours along the spectrum; change with cycle
// 3 means smooth fading along the spectrum; change with time
// 4 means smooth fading along the spectrum, change with cycle.
void assign_colours(byte r1, byte g1, byte b1,
					byte r2, byte g2, byte b2,
					byte r3, byte g3, byte b3,
					byte fade_style, byte num_increments) {

  if (anim_count == 0) {
    colours[BG_RED] = r1;
    colours[BG_GREEN] = g1;
    colours[BG_BLUE] = b1;
    colours[FG1_RED] = r2;
    colours[FG1_GREEN] = g2;
    colours[FG1_BLUE] = b2;
    colours[FG2_RED] = r3;
    colours[FG2_GREEN] = g3;
    colours[FG2_BLUE] = b3;
    
    colours[FGC_RED] = r2;
    colours[FGC_GREEN] = g2;
    colours[FGC_BLUE] = b2;
  
    colours[FADE_STYLE] = fade_style;
    colours[NUM_INC] = num_increments;
  
    colours[INC_RED] = abs(r2 - r3)/num_increments;  
    colours[INC_GREEN] = abs(g2 - g3)/num_increments;
    colours[INC_BLUE] = abs(b2 - b3)/num_increments;	
    
    if (fade_style != 0) {
      if (colours[INC_RED] == 0 && r2 != r3) {
        colours[INC_RED] = 1;
      }
      if (colours[INC_GREEN] == 0 && g2 != g3) {
        colours[INC_GREEN] = 1;
      }
      if (colours[INC_BLUE] == 0 && b2 != b3) {
        colours[INC_BLUE] = 1;
      }
    }
    
    colours[DIR] = 0;
  }			
}

// This changes the current colour according to the two foreground colours
// and the fade style
void perform_spectrum_shifts() {
  if (colours[FADE_STYLE] != 0) {
    if (colours[FADE_STYLE] == 1 || colours[FADE_STYLE] == 2) {
	  byte ran = random_number(colours[NUM_INC] + 1);
	  
	  if (ran != NUM_INC) {
	    if (colours[FG1_RED] < colours[FG2_RED]) {
	      colours[FGC_RED] = colours[FG1_RED] + colours[INC_RED] * ran;
        } else {
	  	  colours[FGC_RED] = colours[FG1_RED] - colours[INC_RED] * ran;
	  	}
	  } else {
		colours[FGC_RED] = colours[FG2_RED];
	  }
	  
	  if (ran != NUM_INC) {
	    if (colours[FG1_GREEN] < colours[FG2_GREEN]) {
	      colours[FGC_GREEN] = colours[FG1_GREEN] + colours[INC_GREEN] * ran;
        } else {
		  colours[FGC_GREEN] = colours[FG1_GREEN] - colours[INC_GREEN] * ran;
	    }
	  } else {
		colours[FGC_GREEN] = colours[FG2_GREEN];
	  }
	  
	  if (ran != NUM_INC) {
	    if (colours[FG1_BLUE] < colours[FG2_BLUE]) {
	      colours[FGC_BLUE] = colours[FG1_BLUE] + colours[INC_BLUE] * ran;
        } else {
		  colours[FGC_BLUE] = colours[FG1_BLUE] - colours[INC_BLUE] * ran;
	    }
	  } else {
		colours[FGC_BLUE] = colours[FG2_BLUE];
	  }
    } else if (colours[FADE_STYLE] == 3 || colours[FADE_STYLE] == 4) {
	  if (colours[DIR] == 0) {
		if (colours[NUM_INC] == 1) {
		  colours[DIR] = 1;
		  colours[FGC_RED] = colours[FG2_RED];
		  colours[FGC_GREEN] = colours[FG2_GREEN];
		  colours[FGC_BLUE] = colours[FG2_BLUE];
		} else {
		  if (colours[FG1_RED] < colours[FG2_RED]) {
		    if (colours[FG2_RED] - colours[FGC_RED] < colours[INC_RED]) {
	          colours[FGC_RED] = colours[FG2_RED];
	          colours[DIR] = 1;
	        } else {
			  colours[FGC_RED] += colours[INC_RED];
		    }
          } else {
	  	    if (colours[FGC_RED] - colours[FG2_RED] < colours[INC_RED]) {
	          colours[FGC_RED] = colours[FG2_RED];
	          colours[DIR] = 1;
	        } else {
			  colours[FGC_RED] -= colours[INC_RED];
		    }
	  	  }
	  	
	  	  if (colours[FG1_GREEN] < colours[FG2_GREEN]) {
		    if (colours[FG2_GREEN] - colours[FGC_GREEN] < colours[INC_GREEN]) {
	          colours[FGC_GREEN] = colours[FG2_GREEN];
	          colours[DIR] = 1;
	        } else {
			  colours[FGC_GREEN] += colours[INC_GREEN];
		    }
          } else {
	  	    if (colours[FGC_GREEN] - colours[FG2_GREEN] < colours[INC_GREEN]) {
	          colours[FGC_GREEN] = colours[FG2_GREEN];
	          colours[DIR] = 1;
	        } else {
			  colours[FGC_GREEN] -= colours[INC_GREEN];
		    }
	  	  }
	  	
	  	  if (colours[FG1_BLUE] < colours[FG2_BLUE]) {
		    if (colours[FG2_BLUE] - colours[FGC_BLUE] < colours[INC_BLUE]) {
	          colours[FGC_BLUE] = colours[FG2_BLUE];
	          colours[DIR] = 1;
	        } else {
			  colours[FGC_BLUE] += colours[INC_BLUE];
		    }
          } else {
	  	    if (colours[FGC_BLUE] - colours[FG2_BLUE] < colours[INC_BLUE]) {
	          colours[FGC_BLUE] = colours[FG2_BLUE];
	          colours[DIR] = 1;
  	        } else {
			  colours[FGC_BLUE] -= colours[INC_BLUE];
		    }
	  	  }
	    }
	  } else if (colours[DIR] == 1) {
		if (colours[NUM_INC] == 1) {
		  colours[DIR] = 0;
		  colours[FGC_RED] = colours[FG1_RED];
		  colours[FGC_GREEN] = colours[FG1_GREEN];
		  colours[FGC_BLUE] = colours[FG1_BLUE];
		} else {
		  if (colours[FG2_RED] < colours[FG1_RED]) {
		    if (colours[FG1_RED] - colours[FGC_RED] < colours[INC_RED]) {
	          colours[FGC_RED] = colours[FG1_RED];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_RED] += colours[INC_RED];
		    }
          } else {
	  	    if (colours[FGC_RED] - colours[FG1_RED] < colours[INC_RED]) {
	          colours[FGC_RED] = colours[FG1_RED];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_RED] -= colours[INC_RED];
		    }
	  	  }
	  	
	  	  if (colours[FG2_GREEN] < colours[FG1_GREEN]) {
		    if (colours[FG1_GREEN] - colours[FGC_GREEN] < colours[INC_GREEN]) {
	          colours[FGC_GREEN] = colours[FG1_GREEN];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_GREEN] += colours[INC_GREEN];
		    }
          } else {
	  	    if (colours[FGC_GREEN] - colours[FG1_GREEN] < colours[INC_GREEN]) {
	          colours[FGC_GREEN] = colours[FG1_GREEN];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_GREEN] -= colours[INC_GREEN];
		    }
	  	  }
	  	
	  	  if (colours[FG2_BLUE] < colours[FG1_BLUE]) {
		    if (colours[FG1_BLUE] - colours[FGC_BLUE] < colours[INC_BLUE]) {
	          colours[FGC_BLUE] = colours[FG1_BLUE];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_BLUE] += colours[INC_BLUE];
		    }
          } else {
	  	    if (colours[FGC_BLUE] - colours[FG1_BLUE] < colours[INC_BLUE]) {
	          colours[FGC_BLUE] = colours[FG1_BLUE];
	          colours[DIR] = 0;
	        } else {
			  colours[FGC_BLUE] -= colours[INC_BLUE];
		    }
	  	  }
	    }
	  }	 	  
	}
  }
}

// Not my code; copied from Wikipedia page on XORShift algorithms
// Maximum is exclusive
// All this does is generate a random number between 0 and the maximum
byte random_number(byte maximum) {
  static uint32_t x = 123456789;
  static uint32_t y = 362436069;
  static uint32_t z = 521288629;
  static uint32_t w = 88675123;
  uint32_t t;

  t = x ^ (x << 11);
  x = y; y = z; z = w;
  w = w ^ (w >> 19) ^ (t ^ (t >> 8));
  return w%maximum;
}


// ============= Animation Functions ===================================

/*
 * It is extremely difficult to explain what all these functions do and
 * what their parameters do.  I've tried my, but I suggest what you do 
 * is to just try them out and see what they do.
 * 
 * These all have 3 common parameters:
 * period:  This sets how quickly the animations update.  Low period
 * 			means fast, high period means slow.
 * fade_in: This sets how quickly the LEDs fade in.  Refer to the
 * 			'perform_fades' function's comments for more info
 * fade_out: This sets how quickly the LEDs fade out.
 */


// Switch all the LEDs onto the current foreground colour.
void all_on(byte fade_in, byte fade_out) {
  led_set_all(colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
  auto_advance_counter++;
  off_speed = fade_out;
}

// Fades all LEDs on and off repeatedly
void fades(byte period, byte fade_in, byte fade_out) {
  
  if (anim_count == 0) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
  }
  
  if (anim_count == 1) {
	led_set_all(colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
  }
  else if (anim_count == period) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	perform_spectrum_shifts();
  }
  
  if (anim_count >= 2 * period) {
	anim_count = 0;
  }
  anim_count++;
  auto_advance_counter++;
  off_speed = fade_out;
}

// This cannot be set with a fade_out of 0 and with the 'wait' flag set.
// It would look odd anyway.
// This is arguably messier than perform_fades()... :(
// If the wait flag is set (to 1) then only 1 LED can be on at a time
void runners(byte period, int8_t dir, byte fade_in, byte fade_out, 
	byte wait, byte bounce) {
  
  static int8_t led_num;
  static int8_t reversed;
  
  if (anim_count == 0) {
	reversed = 1;
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
  }
  
  if (anim_count%period == 0) {
	if (led_num == NUM_LED) {
	  if (wait == 1) {
		if (dir*reversed == 1) {
	      if (get_led_red(NUM_LED - 1) == colours[BG_RED] 
	        && get_led_green(NUM_LED - 1) == colours[BG_GREEN]  
	        && get_led_blue(NUM_LED - 1) == colours[BG_BLUE] ) {
			
			led_num = 0;
			if (bounce == 1) {
			  reversed = -1;
			  if (wait == 0) {
				led_num = 1;
			  }
			}
	      }
	    } else {
		  if (get_led_red(0) == 0 
	        && get_led_green(0) == 0 
	        && get_led_blue(0) == 0) {
		  
		    led_num = 0;
		    if (bounce == 1) {
			  reversed = 1;
			  if (wait == 0) {
				led_num = 1;
			  }
			}
	      }
		}
      } else {
		led_num = 0;
		if (bounce == 1) {
		  reversed *= -1;
		  if (wait == 0) {
		    led_num = 1;
		  }
		}
	  }
	  if (colours[FADE_STYLE] == 2 || colours[FADE_STYLE] == 4) {
		perform_spectrum_shifts();
	  }
	}
	if (led_num != NUM_LED) {
	  if (dir*reversed == -1) {
	    led_set_new(NUM_LED - led_num - 1, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  } else {
		led_set_new(led_num, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  }
	  
	  if (colours[FADE_STYLE] == 1 || colours[FADE_STYLE] == 3) {
		perform_spectrum_shifts();
	  }
	  
	  led_num++;
	}	
  }
  for (loop_var = 0; loop_var < NUM_LED; loop_var++) {
    if (test_not_fading(loop_var)) {
	  if (fade_out == 0) {
	    if (dir*reversed == -1 && loop_var != NUM_LED - led_num) {
		  led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	    } else if (dir*reversed == 1 && loop_var != led_num - 1) {
		  led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	    }
      } else {
		led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	  }
    }
  }
  
  anim_count++;
  auto_advance_counter++;
  off_speed = fade_out;
}

// This does a coundown effect.
// start_state indicates whether the LEDs are all on to begin with (1) and count down by fading out,
// if they're all off to begin with (0) and count down by fading up,
// or just leave them as they are (1).
// dir is the direction in which the LEDs switch off (i.e. left to right or vice versa) and takes either -1 or 1.
// If the wait flag is set (to 1) then an LED needs to be off completely before the next one starts fading.
// Otherwise it will start fading after min_period
// If loop_cycle is set (to 1) then the countdown will repeat, otherwise it will only happen once.
// switch_dir_on_loop (set to 1) switches dir each time a countdown cycle is complete.
// 
// If swap_state_on_loop, loop_cycle and switch_dir_on_loop are set, then you can get
// A sort of bouncing effect.
void counting(byte min_period, int8_t dir, byte fade_up, byte fade_out, 
    byte start_state, byte wait, byte loop_cycle, byte switch_dir_on_loop, 
    byte swap_state_on_loop) {
	
  static int8_t led_num;
  static int8_t reversed;
  static byte continu;
  static byte fade_in;
  static byte state;

  if (anim_count == 0) {
	reversed = 1;
 	led_num = 0;
 	fade_in = fade_up;
 	state = start_state;
	if (state == 1) {
	  led_set_all(colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	} else if (state == 0) {
	  led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	}
  }    

  if (led_num <= NUM_LED) {  
    
    continu = 0;
    
    // If wait flag is set, test for previous LED having finished fading.    
    if (led_num == 0) {
	  byte sum = 0;
    
	  for (loop_var = 0; loop_var < NUM_LED; loop_var++) {
		sum += test_not_fading(loop_var);
	  }
	  
	  if (sum == NUM_LED) {
		continu = 1;
	  }
    } else if (anim_count >= 1 && wait == 1) {
      if (dir * reversed == 1) {
	    if (test_not_fading(led_num - 1)) {
	      continu = 1;
	    }
      } else if (dir * reversed == -1) {
	    if (test_not_fading(NUM_LED - led_num + 1)) {  
	      continu = 1;
	    }
      }
	} else if (wait == 0) {
	  continu = 1;
	}  
  
    if (anim_count >= min_period && continu == 1) {
	  if (dir * reversed == 1) {
	    if (state == 1) {
		  led_set_new(led_num, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	    } else {
		  led_set_new(led_num, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	    }
	  } else {
	    if (state == 1) {
		  led_set_new(NUM_LED - led_num, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	    } else {
		  led_set_new(NUM_LED - led_num, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	    }
	  }
	  anim_count = 0;
      led_num++;
      if (colours[FADE_STYLE] == 1 || colours[FADE_STYLE] == 3) {
		perform_spectrum_shifts();
	  }
    }
    anim_count++;
    auto_advance_counter++;
  } else if (loop_cycle == 1) {
	  
	if ((colours[FADE_STYLE] == 2 || colours[FADE_STYLE] == 4) && swap_state_on_loop != 1) {
	  perform_spectrum_shifts();
	}
	
	if (switch_dir_on_loop == 1) {
	  reversed *= -1;
    }
    
	if (swap_state_on_loop == 1) {
	  if (state == 0) {
		state = 1;
	  } else {
		state = 0;
		if (colours[FADE_STYLE] == 2 || colours[FADE_STYLE] == 4) {
	      perform_spectrum_shifts();
	    }
	  }
    } else {
	  if (state == 1) {
	    led_set_all(colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  } else if (state == 0) {
	    led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	  }
	}
    
 	led_num = 0;
  }
off_speed = fade_out;
}

// This switches on LEDs at random.
// number_on is the number of LEDs switched on per cycle.
// Setting the wait flag (to 1) means only 'number_on' LEDs can be on at once.
void raindrops(byte min_period, byte number_on, byte fade_in, byte fade_out, byte wait) {
  static int8_t led_num;
  static int8_t continu;
  
  if (anim_count == 0) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	continu = 0;
	for (loop_var = 0; loop_var < number_on; loop_var++) {
	  led_num = random_number(NUM_LED);
	  led_set_new(led_num, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
    }
  }
  
  if (anim_count >= min_period) {
	if (test_not_fading(led_num)) {
	  led_set_new(led_num, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	  
	  if (wait == 0 || continu == 1) {
		for (loop_var = 0; loop_var < number_on; loop_var++) {
	      led_num = random_number(NUM_LED);
	      led_set_new(led_num, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
        }
                perform_spectrum_shifts();
		anim_count = 0;
		continu = 0;
	  } else if (wait == 1) {			  
		continu = 1;
	  }
	}
  }
  anim_count++;
  auto_advance_counter++;
  off_speed = fade_out;
}

// pattern_i is a binary value where 1 represents an LED on and a 0 an LED off.
// Pattern invert then simply swaps LEDs that are on to LEDs that are off and vice versa
void pattern_invert(uint16_t pattern_i, byte period, byte fade_in, byte fade_out) {
  static uint16_t pattern;
  
  if (anim_count == 0) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	pattern  = pattern_i;
  }
  
  if (anim_count >= period) {
    for (loop_var = 0; loop_var < NUM_LED; loop_var++) {
	  if (pattern & _BV(loop_var)) {
	    led_set_new(loop_var, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  } else {
	    led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	  }
    }
    anim_count = 0;
    pattern = ~pattern;
  }
  anim_count++;
  auto_advance_counter++;
  perform_spectrum_shifts();
  off_speed = fade_out;
}

// Pattern shift is like pattern_invert, but instead with this function,
// the pattern is shifted along by one.
// If the bounce flag is set (to 1), when the pattern reaches the first or last LED,
// the direction it shifts will be reversed.
void pattern_shift(uint16_t pattern_i, byte period, byte fade_in, byte fade_out, int8_t dir_i, byte bounce) {
  static uint16_t pattern;
  static int8_t dir;
  
  if (anim_count == 0) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	pattern  = pattern_i;
	dir = dir_i;
  }
  
  if (anim_count >= period) {
    for (loop_var = 0; loop_var < NUM_LED; loop_var++) {
	  if (pattern & _BV(loop_var)) {
	    led_set_new(loop_var, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  } else {
	    led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	  }
    }
    anim_count = 0;
    
    if (bounce == 1) {
	  if (dir == 1) {
		if (pattern & _BV(NUM_LED - 1)) {
		  dir = -1;
		}
	  } else if (dir == -1) {
		if (pattern & 1) {
		  dir = 1;
		}
	  }
	}
    
    byte new_bit = 0;
    if (dir == 1) {
	  new_bit = (pattern >> (NUM_LED - 1)) & 1;
	  pattern <<= 1;
	  pattern |= new_bit;
	} else {
	  new_bit = pattern & 1;
	  pattern >>= 1;
	  pattern |= new_bit << (NUM_LED - 1);
	}
  }
  anim_count++;
  perform_spectrum_shifts();
  auto_advance_counter++;
  off_speed = fade_out;
}

// I just did this for fun; it doesn't look very good.
void binary_counter(byte period, byte fade_in, byte fade_out) {
  static uint16_t pattern;
  static int8_t dir;
  
  if (anim_count == 0) {
	led_set_all(colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_in);
	dir = 1;
  }
  
  if (anim_count >= period) {
    for (loop_var = 0; loop_var < NUM_LED; loop_var++) {
	  if (pattern & _BV(loop_var)) {
	    led_set_new(loop_var, colours[FGC_RED], colours[FGC_GREEN], colours[FGC_BLUE], fade_in);
	  } else {
	    led_set_new(loop_var, colours[BG_RED], colours[BG_GREEN], colours[BG_BLUE], fade_out);
	  }
    }
    anim_count = 0;
    
    pattern += dir;
    
    if (pattern >= (1 << NUM_LED) - 1 || pattern <= 0) {
	  dir *= -1;
	}
  }
  anim_count++;
  auto_advance_counter++;
  off_speed = fade_out;
}

