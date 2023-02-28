# This is a message to hold number of ticks from the wheel Encoders
# Use an array of size two of type int32 for the two encoders.
# int32 is used instead of int64 because it is not supporte by Arduino/Teensy.
# An overflow is also unlikely with the Pololu Metal Gearmotor 25Dx67L mm LP 
# 6V with 48 CPR Encoder with its encoder because of its low encoder resolution
int32[2] ticks