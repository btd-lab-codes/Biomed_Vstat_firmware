import time
from machine import I2C, SPI, Pin

# --- Hardware Pin & Address Setup ---
I2C_SDA_PIN = 21
I2C_SCL_PIN = 22

SPI_SCLK_PIN = 18
SPI_MISO_PIN = 19
SPI_MOSI_PIN = 23
SPI_CS_PIN   = 5

DAC_ADDR = 0x60   # MCP4725
LMP_ADDR = 0x48   # LMP91000 AFE

VREF_SYS = 3.3        # ESP32 / DAC Supply Voltage (V)
VREF_ADC = 2.048      # ADS1220 Internal Reference Voltage (V)
R_DUMMY  = 10000.0    # 10 kOhm Precision Resistor "Dummy Cell"

# R_TIA must match the register value;
R_TIA    = 14000.0    # LMP91000 TIA Gain resistor (14 kΩ, matches TIACN=0x13)
CSV_FILENAME = "sample_scan.csv"

# --- Initialize Buses ---
i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=50000)

cs = Pin(SPI_CS_PIN, Pin.OUT, value=1)
spi = SPI(1, baudrate=50000, polarity=0, phase=1,
          sck=Pin(SPI_SCLK_PIN), mosi=Pin(SPI_MOSI_PIN), miso=Pin(SPI_MISO_PIN))

# MENB is active-LOW;
# disables LMP91000 I²C — all register writes silently fail without this line.
menb = Pin(0, Pin.OUT, value=0)

def set_dac_voltage(voltage):
    """Sets MCP4725 DAC output voltage."""
    if voltage < 0: voltage = 0.0
    if voltage > VREF_SYS: voltage = VREF_SYS

    dac_code = int((voltage / VREF_SYS) * 4095)
    buf = bytearray([(dac_code >> 8) & 0x0F, dac_code & 0xFF])
    i2c.writeto(DAC_ADDR, buf)

def init_lmp91000():
    """Configures LMP91000 for 3-Electrode Mode driven by MCP4725 DAC."""
    try:
        # Unlock registers (Write 0x00 to Register 0x01)
        i2c.writeto_mem(LMP_ADDR, 0x01, b'\x00')
        time.sleep_ms(10)

        # LMP91000 has no 10 kΩ gain option; 14 kΩ is the closest above 10 kΩ.
        # At max cell current ~50 µA: VOUT = 50 µA × 14 kΩ = 0.70 V < 2.048 V VREF ✓
        i2c.writeto_mem(LMP_ADDR, 0x10, b'\x13')
        time.sleep_ms(10)

        # REFCN (Reg 0x11) -> Bit 7 = 1 (External DAC Ref Mode: 0x80)
        i2c.writeto_mem(LMP_ADDR, 0x11, b'\x80')
        time.sleep_ms(10)

        # MODECN (Reg 0x12) -> 3-Electrode Amperometric Mode (0x03)
        i2c.writeto_mem(LMP_ADDR, 0x12, b'\x03')
        time.sleep_ms(20)
        print("[LMP91000] Initialized in 3-Electrode External DAC mode.")
    except Exception as e:
        print("[LMP91000] Setup Error:", e)

def init_ads1220():
    """Configures ADS1220: AIN2/AIN3, PGA Bypass, 20 SPS continuous, Internal 2.048V VREF."""
    try:
        cs.value(0)
        time.sleep_us(50)
        spi.write(b'\x06')  # Reset ADS1220
        time.sleep_ms(10)
        cs.value(1)
        time.sleep_ms(10)

        # Reg 0 (0x51): MUX=0101 (AIN2/AIN3 differential), Gain=1, PGA Bypassed
        # Reg 1 (0x04): DR=000 → 20 SPS Normal mode, CM=1 Continuous Conversion
        #               Note: datasheet DR=000 = 20 SPS, NOT 45 SPS as previously commented
        # Reg 2 (0x00): VREF=00 (Internal 2.048 V), no 50/60 Hz FIR filter.
        #               FIR filter MUST be off: read_adc_voltage() issues START/SYNC each call,
        #               which resets the FIR filter state. FIR settling requires ~200 ms (4 × 50 ms),
        #               far longer than the 60 ms ADC wait → RDATA returns 0x000000 while filter settles.
        cs.value(0)
        time.sleep_us(50)
        spi.write(bytearray([0x42, 0x51, 0x04, 0x00]))
        time.sleep_us(50)
        cs.value(1)
        time.sleep_ms(20)
        print("[ADS1220] Registers updated (AIN2/AIN3, PGA Bypass, 20 SPS, Internal 2.048V VREF, no FIR filter).")
    except Exception as e:
        print("[ADS1220] Setup Error:", e)

def read_adc_voltage():
    """Triggers conversion and returns measured voltage on AIN2."""
    cs.value(0)
    time.sleep_us(20)

    spi.write(b'\x08')          # START/SYNC command

    time.sleep_ms(60)           # ≥50 ms for 20 SPS; 60 ms gives 10 ms margin

    spi.write(b'\x10')          # RDATA command
    time.sleep_us(10)
    data = spi.read(3, 0xFF)    # Read 24-bit conversion result
    cs.value(1)

    raw = (data[0] << 16) | (data[1] << 8) | data[2]
    if raw & 0x800000:          # 2's Complement Sign Extension
        raw -= 0x1000000

    v_measured = (raw / 8388607.0) * VREF_ADC
    return v_measured

def execute_lsv_sweep(start_v=0.0, stop_v=0.8, scan_rate_mv_s=50, step_v=0.005):
    """Executes Linear Sweep Voltammetry, logs live data, and exports to CSV."""
    # 1. Initialize AFE and ADC
    init_lmp91000()
    init_ads1220()

    # 2. Set DAC to starting potential and establish zero baseline
    set_dac_voltage(start_v)
    time.sleep(2.0)  # 2 sec stabilization
    v_zero_baseline = read_adc_voltage()
    print(f"[CALIBRATION] Baseline Zero Voltage (V_ZERO): {v_zero_baseline:.4f} V")

    # 3. Step & Delay Parameters
    scan_rate_v_s    = scan_rate_mv_s / 1000.0
    step_delay_ms    = int((step_v / scan_rate_v_s) * 1000)  # e.g. 100 ms at 50 mV/s
    stabilization_ms = int(step_delay_ms * 0.8)              # e.g.  80 ms
    read_delay_ms    = step_delay_ms - stabilization_ms       # e.g.  20 ms

    num_steps = int(abs(stop_v - start_v) / step_v) + 1
    direction = 1 if stop_v >= start_v else -1

    print("\n==========================================================")
    print(f"       RUNNING LSV TEST ON {R_DUMMY/1000:.1f} kOhm DUMMY CELL")
    print("==========================================================")
    print(f"Sweep Range  : {start_v:.3f} V -> {stop_v:.3f} V")
    print(f"Scan Rate    : {scan_rate_mv_s} mV/s (Step: {step_v*1000:.1f} mV)")
    print(f"Step Delay   : {step_delay_ms:.1f} ms")
    print(f"Output File  : '{CSV_FILENAME}'")
    print("----------------------------------------------------------")
    print(f"{'Time (s)':>8} | {'V_Applied (V)':>13} | {'V_ADC (V)':>12} | {'Current (uA)':>12}")
    print("----------------------------------------------------------")

    with open(CSV_FILENAME, "w") as csv:
        csv.write("Time_s,V_Applied_V,V_ADC_V,Current_uA\n")
        t_start = time.ticks_ms()

        for step in range(num_steps):
            v_target = start_v + (step * step_v * direction)

            # Step DAC
            set_dac_voltage(v_target)

            time.sleep_ms(stabilization_ms)

            # Read response
            t_now = time.ticks_diff(time.ticks_ms(), t_start) / 1000.0
            v_adc = read_adc_voltage()

            # Calculate cell current subtracting zero-bias offset:
            # Current = (V_ADC - V_ZERO) / R_TIA
            current_ua = ((v_adc - v_zero_baseline) / R_TIA) * 1e6

            # Output to terminal
            print(f"{t_now:8.3f} | {v_target:13.3f} | {v_adc:12.4f} | {current_ua:12.2f}")

            # Write row to CSV file
            csv.write(f"{t_now:.3f},{v_target:.4f},{v_adc:.4f},{current_ua:.3f}\n")

            time.sleep_ms(read_delay_ms)

    # Return DAC output to 0.0V
    set_dac_voltage(0.0)
    print("----------------------------------------------------------")
    print(f"SUCCESS: LSV Sweep complete! File saved as '{CSV_FILENAME}'.\n")

if __name__ == "__main__":
    execute_lsv_sweep(start_v=0.0, stop_v=0.5, scan_rate_mv_s=50, step_v=0.005)