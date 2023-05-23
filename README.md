# MTRX2700-Escape-Room

This project is a hardware-software interface that demonstrates a proof-of-concept for a 5-minute 'Escape Room', using an integration of STM32F3 microcontrollers, the Pan-Tilt-Unit hardware,
capacitive touch sensors, and other miscellaneous hardware, in Assembly, C and Python code.

### Group members:
- Christopher Fathallah
- Jenny Lin
- Lucas Parker
- Oscar Fevre
- Sarah Lin

### Roles and Responsibilities:
Christopher Fathallah: Joystick and PTU + Failure-to-Complete Buzzer 
- Configured joystick hardware with the STM32F3
- Enabled input/output readings from the joystick to the PTU
- Configured the failure-to-complete buzzer

Jenny Lin: Keypad and Serial I/O
- Configured keypad hardware with STM32F3
- Enabled serial transmission and receiving with USART1 and PuTTY

Lucas Parker: LIDAR and Timers with Interrupts
- Configured LIDAR hardware and readings to the STM32F3
- Enabled timers with interrupts from PTU and LIDAR readings to the STM32F3

Oscar Fevre: Capacitive Touch
- Configured capacitive touch hardware and input to the STM32F3
- Enabled serial output and configured with Python to output sounds

Sarah Lin: Game Timer + Demonstration Manufacture + IMU
- Configured 7-seg display hardware with STM32F3 timer interrupts and integrated failure-to-complete buzzer 
- Manufactured demonstration housing and mounting 
- Enabled output readings from STM32F3 internal gyroscope and compass (module later discarded, unused)

## Section 1 - Joystick and PTU

The code for this module/board can be found under the PTU and Joystick project.

### Joystick Design
This module uses an Arduino joystick to control the PTU unit movement.

#### Setup
To set this up, for the current project the user will need to connect the correct hardware. The following pins describe the connection of the joystick to the STM32.
* GND -> Any GND on STM
* 5V -> 5V STM
* VRx -> PA2
* VRy -> PC0
PA2 and PC0 are pins configured in the HAL of these project. They can be changed in the HAL to any valid ADC pin, so long as they are not already being used.

#### How it works
The analog voltages which are being outputted from the joystick change depending on the angle of the joystick. This is done using 2 potentiometers, one for each direction. NOTE: the scale that the voltage decreases is not linear, this poses challenges for smoothly moving the PTU on the joystick angle. 
The analog voltages are read by ADC pins which convert the voltage range of (0V to 5V) to (0, 4095). These digital values can then be used to map movement of the joystick.

#### Software Elements
A lot of this code cannot be modularised. For instance, everything to do with the HAL needs to stay in the main excutable. For this module, that means reading the ADC values. Code that is modularised is the map that is built. This is found in a supplemtary file, and it updates the x and y servo values of the PTU.

#### Design Choices
To operate the PTU, the stick should be pointed firmly in that direction. This is a purposeful design choice to minimise inaccurate inputs. As explained earlier, the voltages from the joystick are non-linear so making a map is pretty difficult. Libraries could be used for this, but for now the current implementation for the map is found in the "updateValues" function. What this function does is depending on the input, it increments/decrements the servo value by a set amount found in the "INCREMENT_SERVO" constant.

Also, there is a small radius around the joystick's neutral position which will not move the PTU at all. This is a purposeful design choice so that the noise from the jumper cables and the joystick does not move the PTU (otherwise it would have small rapid movements).

#### Testing
First, to verify that the joystick was working a digital oscilloscope was used to find the voltage ranges of each component. Then to see how the STM32F3 reads analog values, polling was used for a single pin. After this was working, both x and y were implemented, and 2 pins had to be chosen which wouldn't clash with any other function of the STM. Now, we need to determine direction. Breakpoints were used to determine the digital extremities of the joystick. Then, a basic linear map would change the servo position based on the joystick. This was very hard to control, but proved the concept would work. This was later refined so that the joystick gave very incremental control of position. 

### PTU Configuration

#### Setup
The PTU is mainly set-up for us using Stewart's basic library shown in "major-project-base", but this section will give a brief overview of how it works.
The following pins describes how the hardware is set-up:
* GND -> Any GND on STM
* GND -> Any GND on STM
* PWM1 -> PA1 (Control x-axis servo motor)
* PWM2 -> PA15 (Control y-axis servo motor)

#### How it Works
Again, this code is consistent with Stewart's library however here is a brief overview of how it works:
The PA1 and PA15 pins on the STM32 output servo positions for the motors. 

#### Software Elements
In this module the values that control the servo are controlled by the joystick. Once these are found, they are output to TIM2 which will be sent to the PTU. The x-value sets CCR2 of TIM2 and the y-value sets CCR1.

#### Design Choices
We have purposefully limited the values of the servo position to be within the range (0, 3000), otherwise we could damage the motors or there could be unexpected behaviour.

#### Testing
This module required a decent amount of testing. A big part of this was poor connections through jumper cables, which would be sending bad signals. Adding to this, depending on the exact build of your PTU, the wiring of the PTU might physically limit the range of motion of the x-servo motor. This can be changed in the code by limiting the X_MAXIMUM_SERO constant from 3000.



## Section 2 - LIDAR and Timers with Interrupts

Again, this section of code is largely influecned by Stewart's project, however we have changed how data is read in.

### LIDAR Inputs
This gives us 2-options to get LIDAR readings. NOTE: Depending on the configuration of your unit, some I2C connection will NOT work, so PWM will have to be used. However, if given the choice, use the I2C values. Our implementation used the I2C values which will be more accurate, however you can easily modify the code to take both.

#### Setup
* SDA -> PB7 (I2C for LiDAR)
* SCL -> PB6 (I2C for LiDAR)
* LASPWM -> PA8 (PWM for LiDAR)

#### Software Elements
To get our LIDAR data we have set-up an interrupt based timer. This timer will collect data every half a second and store that data in an array. After 3-seconds, it will average the values stored in the array. This value is averaged to check if the LIDAR has hit one of the "puzzle spots", or "waypoints". A waypoint is a fixed LIDAR distance that the player has to hit in order to solve this puzzle. To do this, we have laser cut a clear enclosure for the PTU/LIDAR and put tape over the waypoints. 

#### Design Choices
To ensure that the player doesn't hit a waypoint accidentally, thereby triggering a clue, we have implemented a timer that averages the LIDAR value after 3 seconds. NOTE: if a player goes to a waypoint at the very end of a timer it will take an effective 6 seconds for the clue to unlock. This is an intentional choice to esnure that the player is knowingly finding these spots.

We laser cut a transparent box so that the players can see how they are moving the sensor, this is self explanatory. Our PTU housing has fixed tabs that are used to consistently align the sensor to the waypoints. If this wasn't consistent, we'd have poor behaviour.

The timer used to sample LIDAR values could of been faster. For instance, rather than measuring data every 0.5 seconds we could do it every 1/10th of a second. Hence, we'd average 30 values rather than 6. While this seems like it will give more accurate readings, we found it was unnecessary and therefore more efficient to use a timer of 0.5 seconds. It is also important to note that this timer acts as a moving average.

#### Testing
To test the LIDAR values we started with Stewart's base code. From here we learnt that sometimes a power-cycle is necessary, when LIDAR values stop reading. We have taken this into account in our design, and power-cycling can still easily be performed with the designed housing.



## Section 3 - Keypad

#### Setup
The keypad consists of a 4x4 matrix with 4 rows and 4 columns. It utilizes a total of 8 GPIO pins, with 4 pins assigned as output for the keypad rows and 4 pins assigned as input for the keypad columns

* ROW 1 -> PA7
* ROW 2 -> PA6
* ROW 3 -> PA5
* ROW 4 -> PA4
* COLUMN 1 -> PA3
* COLUMN 2 -> PA2
* COLUMN 3 -> PA1
* COLUMN 4 -> PA0
#### How it Works
The process of reading the keys on the keypad involves the following steps. First, the rows of the keypad are pulled LOW sequentially, while the remaining rows are maintained at a HIGH voltage level. This step is essential for isolating and scanning each row individually.Once the rows are pulled LOW, the columns of the keypad are examined. Each column is checked to determine its state. If any particular column is detected as LOW, it signifies that the corresponding key aligned with that column and row combination is pressed.By systematically scanning through the rows and checking the columns, the keypad can effectively identify and register key presses. This method ensures accurate and reliable detection of key inputs on the keypad.
#### Software Elements
The GPIO_PIN_RESET and GPIO_PIN_SET commands are utilized to pull up or down the rows while checking the states of the columns. An if-loop is employed to determine if a specific column is pressed, assigning the corresponding character to the 'key' variable. Another if-else loop is used to verify if the user input matches the password. A while loop implements an infinite loop until the correct password is entered
#### Design Choices
The 4x4 keypad functionality is implemented using GPIO pins. These GPIO pins can be configured as either inputs or outputs, allowing for flexible usage. In the default configuration, the columns of the keypad are set as inputs, while the rows are set as outputs. This arrangement enables the detection of key presses. To locate the pressed characters, the state of the rows is manipulated one row at a time. Each row is set to a specific output level (either 0 or 1), while the columns' state is observed. By systematically scanning through each row and checking the state of the columns, the keypad can identify the pressed characters accurately. By utilizing GPIO pins and manipulating the state of the rows and columns, the 4x4 keypad can effectively detect and locate the key presses, enabling reliable input functionality.
#### Testing
The functionality of the keypad press is tested by employing PUTTY and USART1. PUTTY is a terminal emulation software that allows communication with the microcontroller or system under test. USART1 refers to the Universal Synchronous/Asynchronous Receiver Transmitter interface.

During the test, the key press on the keypad is observed, and the corresponding character is displayed on PUTTY. This process serves to validate the functionality and accuracy of the keypad. By comparing the expected characters with the actual characters received on PUTTY, the validity of the keypad's operation can be determined.

## Section 4 - Capacitive Touch

### Resistors and Capacitors
Two groups conisiting of a capacitor connected to ground and three different resistor values each connected in series with their own wire electrode acting as keys on a piano. In each sampling group, the capacitors are used for sampling while the three resistors and electrodes are used as channels to read and adjust an acquisition value.

#### Setup
* When initialising the project, enable default modes on pins. In the ioc HAL interface select TSC in 'System Core'. Select the 'Sampling' in Group 6 and Group 8 as G6_IO4 and G8_IO4. Tick all the remaining boxes, ignoring 'Shielding'.

* Touch Sensing Group 6
** Sampling Capacitor -> PB14, GND
** C KEY -> PB11
** D KEY -> PB12
** E KEY -> PB13

* Touch Sensing Group 8
** Sampling Capacitor -> PD15, GND
** F KEY -> PD12
** G KEY -> PD13
** A KEY -> PD14

#### How it Works
The process of reading the acquisition values recieved from the sampling channels and converting this to a specific note value requires a number of steps. Firstly the inbuilt HAL functions are used to discharge the capacitors before starting the acquisition process and charging the capaitor once more. The two groups are checked using polling if they have recieved this value which is then stored. This results in each cycle having two acquisition values, group 6 controls the 'C', 'D' and 'E' notes while group 8 controls the 'F', 'G' and 'A' notes. Note values can be registered by checking which tested range the acquisition values for the respective notes lie in. It is importnant to note that false positive values may occur when approahing smaller acquisition values which can be accounted for in the software.

#### Software Elements
Three index and flag values are used to measure if a note has been pressed on purpose or to check if the note is in the sequence and the user's current position in the sequence. Once an acquisition value has been measured and checked to be within a specific note range, the key value is updated with the note and a valid key counter is incremented. This valid key index is then checked to be above a certain threshold which will then output the key note as a uint8_t char to serial using USART, the index is then set to 0 before the next acquisition value is read. 

There is a set pattern of notes which the user must reach in order. If the key is valid, the key is checked against the user's indexed position in the sequence. If the key is correct then the index is incremented and set to 0 if it is not. Once the end of the sequence has been reached, a completion flag is then raised. In the next iteration of the loop, if this flag is raised then a different char is sent through serial and the program exits

#### Design Choices
To validate note presses before they are tranmitted, an index of valid key presses is continually incremented and a note press is only considered valid once it reaches a threshold. This threshold can be adjusted.

#### Testing
Testing mainly consists of adjusting the ranges for the acquisition values due to the sensitivity of the electrodes. This sensivity is coaused by a number of factors, including the wire connection from the electrodes to the STM board, surface area of electrode covered, moisture on finger tips. 

These values can be tested for experimentally by transmitting the acquisition values through serial to the user's terminal.

### Musical Sound with Python
The note once passed to the serial port can then be read in python and played aloud. A victory tune and hint for the next image are loaded when the completion char is received.

#### Setup
The script requires the installation of a number of python packages which can be downloaded in the user's local terminal:
- serial --> pip install pyserial
- musicalbeeps --> pip install musicalbeeps
- playsound --> pip install playsound
- PIL --> pip install pillow

Download the victory tune as a .wav file and the image as .jpg.

Once the board has been flashed with the above C code, the Python code can then be run selecting the correct serial port to read from.

#### How it Works
The char from the C code is received as a byte which is then decoded then passed to a function that checks the note in a list. If the note is in the list then the note plays or in the case of completion, the victory tune is played and the image containing the next hint is loaded.

#### Testing
Testing involved checking each note worked as well as the overall integration between the packages.

## Section 5 - 7-Seg Displays and Buzzer

### 7-Seg Display Configuration
A set of three 7-segment displays were used to visualise the gameplay timer. A piezo buzzer was used to generate a failure-to-complete sound at the end of the game, where players do not complete the game in time.

#### Setup
To set up this module, the user would need to configure the hardware correctly. Output pins are set as follows: 
* 7-Seg 1 (Seconds Units)
** Segment A -> PA1
** Segment B -> PA2
** Segment C -> PA3
** Segment D -> PA4
** Segment E -> PA5
** Segment F -> PA6
** Segment G -> PA7

* 7-Seg 2 (Seconds Tens)
** Segment A -> PC1
** Segment B -> PC2
** Segment C -> PC3
** Segment D -> PC4
** Segment E -> PC5
** Segment F -> PC6
** Segment G -> PC7

* 7-Seg 3 (Minutes) 
** Segment A -> PD1
** Segment B -> PD2
** Segment C -> PD3
** Segment D -> PD4
** Segment E -> PD5
** Segment F -> PD6
** Segment G -> PD7
** DP Segment -> PD8

* All connections to POWER were spliced together and can be connected to any power pin on the STM32F3 discovery board. 

#### How it Works
The 7-segment outputs are configured through an array that maps the 10 digits 0-9 to a hex value arrived at through binary values found using the following truth logic: 
*     | Seg-A | Seg-B | Seg-C | Seg-D | Seg-E | Seg-F | Seg-G
* 0   | 1     | 1     | 1     | 1     | 1     | 1     | 0
* 1   | 0     | 1     | 1     | 0     | 0     | 0     | 0
* 2   | 1     | 1     | 0     | 1     | 1     | 0     | 1
* 3   | 1     | 1     | 1     | 1     | 0     | 0     | 1
* 4   | 0     | 1     | 1     | 0     | 0     | 1     | 1
* 5   | 1     | 0     | 1     | 1     | 0     | 1     | 1
* 6   | 1     | 0     | 1     | 1     | 1     | 1     | 1
* 7   | 1     | 1     | 1     | 0     | 0     | 0     | 0
* 8   | 1     | 1     | 1     | 1     | 1     | 1     | 1
* 9   | 1     | 1     | 1     | 1     | 0     | 1     | 1

The total time is counted and computed in seconds, where the frequency of the STM32F3 clock is set such that every cycle of the clock corresponds to 1 ms, allowing for more intuitive configuration. For every cycle of the clock, the total time is decremented via interrupt until it reaches zero, which then raises a flag signalling the end of the timer; after each decrement, the total time is calculated to output values of minutes, seconds (tens), and seconds (ones) which are then outputted to the display logic of each individual 7-segment display. Every 7-segment display has the same output logic (except for Segment 3 which has an additional input that holds the DP segment at a constant HIGH), which maps the calculated digit to the corresponding index of the character array discussed above; this then flips the required bits to correctly display the timer values. 

The user is able to adjust the display of this module by adapting the number of available characters that can be displayed and inputting the correct hex value of said characters into the character array. The total value of the timer can also be adjusted by the user. 

#### Software Elements
The count down calculations operate off a timer interrupt using TIM2. For every cycle of the clock, which is configured at a frequency that allows the user to equate every cycle to 1 ms, a series of logic blocks counts and redisplays the timer value. 

#### Design Choices
The initial design architecture of this module involved the use of a shift register (74HC595) to drive the logic of the 7-segment displays. This decision was made primarily to decrease the amount of inputs needed for the operation of the timer; the change from direct 7-seg-to-STM32F3 inputs to the use of a shift register to push the bits display by display would have decreased the number of inputs needed from 21 to 3. However, ultimately due to hardware failure of the shift register and a lack of time to re-source the component, this architecture was replaced with direct connections. 

#### Testing
Hardware in this module was tested using LED indicators as well as analogue circuits that were first tested in a simulation software to verify functionality. This was the primary methodology used to establish that the latch and input pins of the shift register were malfunctioning. 

The software and logic of the timer was configured and tested experimentally through the use of specific input conditions and logic conditions. This allowed the determination of specific behaviour in the timer module and the display of the timer. 

### Buzzer Configuration

#### Setup
To set this up for the current project, the user would need to connect the correct hardware. The following pins describe the connection of the piezo buzzer to the STM32F3.
Note: the polarity of the buzzer does not matter, since all that is needed to drive them is a PWM signal.
* Connection 1 -> Any GND on STM32F3
* Connection 2 -> PB1 

#### How it Works
The timer interrupt is relatively simple. Using a software timer (TIM3, which is interrupt driven), a PWM signal can be generated. This signal is outputted over PB1 and drives the frequency of the buzzer. The frequency can be changed by changing the "TIM3-ARR" variable, and the duty cycle can be changed using the variable "TIM3->CCR1". NOTE: CCR1 should also be a fraction of ARR.

#### Software Elements
The buzzer is triggered when the countdown finishes, so there is a flag that is raised whenever there the countdown timer reaches 0. This triggers a state in the forever main loop to turn on the interrupt driving the buzzer. 

#### Design Choices
To properly create this curcuit we needed a MOSFET to properly drive the piezo buzzer. So, the MOSFET would act like a switch, and the logic would flow:
5V -> MOSFET (input: PWM signal from STM, thereby making the MOSFET a switch) -> buzzer
Since, we could not source this part, the buzzer was connected directly from the PWM signal to ground. This could of either; ruined the STM or given an insufficient amount of current, HOWEVER seems to work OK with the current set-up.

#### Testing
Testing was quite simple. The values for the duty cycle and reload functions were changed experimentally to determine a sound that would suit this escape room. The code was tested by connecting an oscilloscope to the output pin before the buzzer.

