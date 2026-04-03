-- Tx and Tx config
NUM_TX            = 3
NUM_RX            = 4
--
START_FREQ        = 60
ADC_START_TIME    = 6
FREQ_SLOPE        = 60.012
ADC_SAMPLES       = 256
SAMPLE_RATE       = 4400
RX_GAIN           = 30

IDLE_TIME         = 7
RAMP_END_TIME     = 65
CHIRP_LOOPS       = 128
PERIODICITY       = 100

local autoStart   = 1

NumOfProfile      = 1
NumOfChirpInLoop  = 3
NumOfFrame        = 0

local profile     = {}
profile[0]        = { 0, START_FREQ, IDLE_TIME, ADC_START_TIME, RAMP_END_TIME, 0, 0, 0, 0, 0, 0, FREQ_SLOPE, 0,
    ADC_SAMPLES,
    SAMPLE_RATE, 0, 0, RX_GAIN }
profile[1]        = { 1, START_FREQ, IDLE_TIME, ADC_START_TIME, RAMP_END_TIME, 0, 0, 0, 0, 0, 0, FREQ_SLOPE, 0,
    ADC_SAMPLES,
    SAMPLE_RATE, 0, 0, RX_GAIN }
profile[2]        = { 2, START_FREQ, IDLE_TIME, ADC_START_TIME, RAMP_END_TIME, 0, 0, 0, 0, 0, 0, FREQ_SLOPE, 0,
    ADC_SAMPLES,
    SAMPLE_RATE, 0, 0, RX_GAIN }
profile[3]        = { 3, START_FREQ, IDLE_TIME, ADC_START_TIME, RAMP_END_TIME, 0, 0, 0, 0, 0, 0, FREQ_SLOPE, 0,
    ADC_SAMPLES,
    SAMPLE_RATE, 0, 0, RX_GAIN }
local ptx         = {}
ptx[1]            = { 1, 0, 0 }
ptx[2]            = { 0, 1, 0 }
ptx[3]            = { 0, 0, 1 }

-- frame config:
-- note: the duration between two chirp
local periodicity = 100

local info        = debug.getinfo(1, 'S')
local file_path   = (info.source)
file_path         = string.gsub(file_path, "@", "")
file_path         = string.gsub(file_path, "\\[%w_]-%.lua$", "\\")
local fw_path     = file_path .. "..\\..\\rf_eval_firmware"

COM_PORT          = 9
ar1.FullReset()
RSTD.Sleep(2000)
ar1.SOPControl(2)
RSTD.Sleep(1000)
-- Baud Rate
ar1.Connect(COM_PORT, 921600, 1000)
RSTD.Sleep(2000)
-- Start Frequency depend on radar
ar1.frequencyBandSelection("60G")
local bitopfile = file_path .. "\\" .. "bitoperations.lua"
dofile(bitopfile)

BSS_FW = fw_path .. "\\radarss\\xwr68xx_radarss.bin"
MSS_FW = fw_path .. "\\masterss\\xwr68xx_masterss.bin"

assert(ar1.DownloadBSSFw(BSS_FW) == 0)
RSTD.Sleep(2000)

assert(ar1.DownloadMSSFw(MSS_FW) == 0)
RSTD.Sleep(2000)

assert(ar1.PowerOn(1, 1000, 0, 0) == 0)
RSTD.Sleep(1000)

assert(ar1.RfEnable() == 0)
RSTD.Sleep(1000)

assert(ar1.ChanNAdcConfig(1, 1, 1, 1, 1, 1, 1, 2, 1, 0) == 0)
RSTD.Sleep(1000)

assert(ar1.LPModConfig(0, 0) == 0)
assert(ar1.RfLdoBypassConfig(0x3) == 0)
RSTD.Sleep(2000)

assert(ar1.RfInit() == 0)
RSTD.Sleep(1000)

assert(ar1.DataPathConfig(1, 1, 0) == 0)
RSTD.Sleep(1000)

assert(ar1.LvdsClkConfig(1, 1) == 0)
RSTD.Sleep(1000)

assert(ar1.LVDSLaneConfig(0, 1, 1, 0, 0, 1, 0, 0) == 0)
RSTD.Sleep(1000)


for n = 0, NumOfProfile - 1 do
    assert(ar1.ProfileConfig(profile[n][1], profile[n][2], profile[n][3], profile[n][4], profile[n][5], profile[n][6],
        profile[n][7], profile[n][8], profile[n][9], profile[n][10], profile[n][11], profile[n][12], profile[n][13],
        profile[n][14], profile[n][15], profile[n][16], profile[n][17], profile[n][18]) == 0)
end

for i = 0, NumOfChirpInLoop - 1 do
    local p = 0
    local tx1 = ptx[i + 1][1]
    local tx2 = ptx[i + 1][2]
    local tx3 = ptx[i + 1][3]
    assert(ar1.ChirpConfig(i, i, p, 0, 0, 0, 0, tx1, tx2, tx3) == 0)
end
RSTD.Sleep(1000)

assert(ar1.FrameConfig(0, NumOfChirpInLoop - 1, NumOfFrame, CHIRP_LOOPS, periodicity, 0, 0, 1) == 0)
RSTD.Sleep(1000)

assert(ar1.SelectCaptureDevice("DCA1000") == 0)
RSTD.Sleep(1000)

assert(ar1.CaptureCardConfig_EthInit("192.168.33.30", "192.168.33.180", "12:34:56:78:90:12", 4096, 4098) == 0)
RSTD.Sleep(1000)

assert(ar1.CaptureCardConfig_Mode(1, 2, 1, 2, 3, 30) == 0)
RSTD.Sleep(1000)

assert(ar1.CaptureCardConfig_PacketDelay(25) == 0)
RSTD.Sleep(1000)

local data_path    = file_path .. "D:\\Projects\\py\\mmwave-radar-tutorial"

local adc_data_path = data_path .. "\\" .. "\\adc_data.bin"
ar1.CaptureCardConfig_StartRecord(adc_data_path, 0)
RSTD.Sleep(1000)

if (autoStart > 0) then
    ar1.StartFrame()
    RSTD.Sleep(2000)
end
