
# Static Config
numTxAntennas = 3
numRxAntennas = 4

# Profile Config
StartFreq  = 60e9 # Hz
numADCSamples = 256
IdleTime = 7e-6 # s
ADCStartTime = 6e-6 # s
RampEndTime = 65e-6 # s


# Data Config
# complex 16bit = 2 bytes
# I/Q = complex * 2 = 4 bytes
IQ = 4

# Chirp Config

# Frame Config
# Tx0 -> Tx2 -> Tx1


# other
C = 3e8 # m/s
WaveLength = C / StartFreq