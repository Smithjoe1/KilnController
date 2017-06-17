# KilnController
Arduino powered kiln controller, it would also work for reflow ovens.

Current runs on a Mega 2560 but can easily fit on a UNO with some small changes to the number of profiles and profile stages stored in the EEPROM.

Uses reprap smart full lcd controller as the interface.

Uses a MAX31855 thermocouple but can easily be changed to use other interfaces. For kiln usage the Thermocouple interface needs to read up to 1300 Degrees C.

Switches two Relays to control power flow to the heater coils. There is scope to change this to a solid state relay and implement PID control but relays or contactors are recommended for continuous loads over 15A

Allows the user to create temperature control programs by setting a number of stages, either higher or lower than the previous stage, control the change of temperature in degrees C per hour and how long each stage will hold in minutes before moving to the next stage of the program.

All programs can be run directly or stored in EEPROM for later use.

Uses U8glib and Mt2klib to handle screen and interface.

The main screen allows the user to begin, preview or edit the program. It also shows the current profile loaded, the highest temp it will reach and the current thermocouple reading.

The edit profile screen allows the user to edit, save or load profiles.

The individual edit profile screen allows the user to change each of the program stages.
Stage: The controller runs through this sequentially once it reaches the temperature defined below and has held for a set period of time.
Temp: The target temperature in Degrees C.
Rate: The speed in Degrees C per hour to climb to the target temperature.
Hold: How many minutes to hold at the target temperature before moving to the next stage.

Save/Load profile
Select the profile number you wish to load or save. The load profile gives you a preview of the maximum temperature for the profile so you can quickly see what you will be loading.

Preview profile.
This screen outputs all the profile stages to see the stages up or down past the peak temperature quickly. If you change the code to have less profile stages then you will have to remove some of these hard coded links. 

Begin Firing.
This is a final screen and currently the only way to stop the firing is to remove power from the arduino.
It shows the current thermocouple temperature reading. 
The highest temperature the program has recorded. 
How long the program has been running for. 
The currently loaded profile and the current stage.
The current target tmeperature and the peak temperature of the stage.
Finally the program draws two graphs, one for the target temperature and another overlapping graph on how close the kiln was able to reach these temperatures. This gives insight to if the kiln is not coming up to temperature properly.
