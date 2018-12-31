# CTCSS_Tone_Generator

 This program is generates CTCSS tones.

 Circuit info:
 The output is PWM and must be passed though low pass filters (I used 2 RC filters) to create a sine wave, then buffered with an op amp, followed by a 
 10k trim put to set the final output level.
 
 Suggested filter cut-off frequency is 300Hz
 
 Frequency response can be flattened by adjusting variables in the generateSineWave() procedure.

 Copyright 2018 by Dennis Cabell
 KE8FZX