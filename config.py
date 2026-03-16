
# Static Config
numTx= 3
numRx= 4

# Profile Config
StartFreq  = 60e9 # Hz
numADCSamples = 256
IdleTime = 7e-6 # s
ADCStartTime = 6e-6 # s
RampEndTime = 65e-6 # s

ADCSampleRate = 4.4e6
FreqSlope = 60.012e12
IdleTime = 7e-6

# Data Config
IQ = 2

# Chirp Config
numChirpLoops = 128

# Frame Config
# Tx0 -> Tx1 -> Tx2
Periodicity = 0.1  # s

# other
C = 3e8 # m/s
WaveLength = C / StartFreq