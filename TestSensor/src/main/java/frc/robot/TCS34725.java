package frc.robot;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;

import java.util.HashMap;

/**
 * Driver for the TCS34725 RGB color sensor. Adapted for roboRio/WPILib
 * based on code from  <a href="https://github.com/OlivierLD/raspberry-coffee/blob/master/I2C.SPI/src/i2c/sensor/TCS34725.java">raspberry-coffee</a>.
 * This code was tested using the <a href="https://www.adafruit.com/product/1334">Adafruit color sensor.</a>
 * Connect sensor to roboRio I2C port and instantiate class
 *
 * @author Chuck Benedict, Mentor, Team 997
 *
 *
 * @author Improvements - Robert Hilton, Mentor, Team 195, 2020
 * Converted to synchronous, non-blocking code, except for init
 * Remove ByteBuffer dynamic allocation
 */
public class TCS34725 {
	private I2C i2c;

	public final static int TCS34725_COMMAND_BIT = 0x80;
	public final static int TCS34725_COMMAND_AUTO_INCREMENT = 0x20;
	public final static int TCS34725_ADDRESS = 0x29;
	public final static int TCS34725_ENABLE = 0x00;
	public final static int TCS34725_ENABLE_AIEN = 0x10; // RGBC Interrupt Enable
	public final static int TCS34725_ENABLE_WEN = 0x08; // Wait enable - Writing 1 activates the wait timer
	public final static int TCS34725_ENABLE_AEN = 0x02; // RGBC Enable - Writing 1 actives the ADC, 0 disables it
	public final static int TCS34725_ENABLE_PON = 0x01; // Power on - Writing 1 activates the internal oscillator, 0 disables it
	public final static int TCS34725_ATIME = 0x01; // Integration time
	public final static int TCS34725_WTIME = 0x03; // Wait time (if TCS34725_ENABLE_WEN is asserted)
	public final static int TCS34725_WTIME_2_4MS = 0xFF; // WLONG0 = 2.4ms   WLONG1 = 0.029s
	public final static int TCS34725_WTIME_204MS = 0xAB; // WLONG0 = 204ms   WLONG1 = 2.45s
	public final static int TCS34725_WTIME_614MS = 0x00; // WLONG0 = 614ms   WLONG1 = 7.4s
	public final static int TCS34725_AILTL = 0x04; // Clear channel lower interrupt threshold
	public final static int TCS34725_AILTH = 0x05;
	public final static int TCS34725_AIHTL = 0x06; // Clear channel upper interrupt threshold
	public final static int TCS34725_AIHTH = 0x07;
	public final static int TCS34725_PERS = 0x0C; // Persistence register - basic SW filtering mechanism for interrupts
	public final static int TCS34725_PERS_NONE = 0b0000; // Every RGBC cycle generates an interrupt
	public final static int TCS34725_PERS_1_CYCLE = 0b0001; // 1 clean channel value outside threshold range generates an interrupt
	public final static int TCS34725_PERS_2_CYCLE = 0b0010; // 2 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_3_CYCLE = 0b0011; // 3 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_5_CYCLE = 0b0100; // 5 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_10_CYCLE = 0b0101; // 10 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_15_CYCLE = 0b0110; // 15 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_20_CYCLE = 0b0111; // 20 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_25_CYCLE = 0b1000; // 25 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_30_CYCLE = 0b1001; // 30 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_35_CYCLE = 0b1010; // 35 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_40_CYCLE = 0b1011; // 40 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_45_CYCLE = 0b1100; // 45 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_50_CYCLE = 0b1101; // 50 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_55_CYCLE = 0b1110; // 55 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_PERS_60_CYCLE = 0b1111; // 60 clean channel values outside threshold range generates an interrupt
	public final static int TCS34725_CONFIG = 0x0D;
	public final static int TCS34725_CONFIG_WLONG = 0x02; // Choose between short and long (12x) wait times via TCS34725_WTIME
	public final static int TCS34725_CONTROL = 0x0F; // Set the gain level for the sensor
	public final static int TCS34725_ID = 0x12; // 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727
	public final static int TCS34725_STATUS = 0x13;
	public final static int TCS34725_STATUS_AINT = 0x10; // RGBC Clean channel interrupt
	public final static int TCS34725_STATUS_AVALID = 0x01; // Indicates that the RGBC channels have completed an integration cycle

	public final static int TCS34725_CDATAL = 0x14; // Clear channel data
	public final static int TCS34725_CDATAH = 0x15;
	public final static int TCS34725_RDATAL = 0x16; // Red channel data
	public final static int TCS34725_RDATAH = 0x17;
	public final static int TCS34725_GDATAL = 0x18; // Green channel data
	public final static int TCS34725_GDATAH = 0x19;
	public final static int TCS34725_BDATAL = 0x1A; // Blue channel data
	public final static int TCS34725_BDATAH = 0x1B;

	public final static int TCS34725_INTEGRATIONTIME_2_4MS = 0xFF;   //  2.4ms - 1 cycle    - Max Count: 1024
	public final static int TCS34725_INTEGRATIONTIME_24MS = 0xF6;   // 24ms  - 10 cycles  - Max Count: 10240
	public final static int TCS34725_INTEGRATIONTIME_50MS = 0xEB;   //  50ms  - 20 cycles  - Max Count: 20480
	public final static int TCS34725_INTEGRATIONTIME_101MS = 0xD5;   //  101ms - 42 cycles  - Max Count: 43008
	public final static int TCS34725_INTEGRATIONTIME_154MS = 0xC0;   //  154ms - 64 cycles  - Max Count: 65535
	public final static int TCS34725_INTEGRATIONTIME_700MS = 0x00;   //  700ms - 256 cycles - Max Count: 65535

	public final static int TCS34725_GAIN_1X = 0x00;   //  1x gain
	public final static int TCS34725_GAIN_4X = 0x01;   //  4x gain
	public final static int TCS34725_GAIN_16X = 0x02;   //  16x gain
	public final static int TCS34725_GAIN_60X = 0x03;   //  60x gain

	public final static HashMap<Integer, Double> INTEGRATION_TIME_DELAY = new HashMap<>();

	static {
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_2_4MS, 0.0024);   // 2.4ms - 1 cycle    - Max Count: 1024
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_24MS, 0.024);   // 24ms  - 10 cycles  - Max Count: 10240
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_50MS, 0.050);   // 50ms  - 20 cycles  - Max Count: 20480
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_101MS, 0.101);   // 101ms - 42 cycles  - Max Count: 43008
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_154MS, 0.154);   // 154ms - 64 cycles  - Max Count: 65535
		INTEGRATION_TIME_DELAY.put(TCS34725_INTEGRATIONTIME_700MS, 0.700);   // 700ms - 256 cycles - Max Count: 65535
	}

	public static final int kDeviceID = 0x44;

	public final static int INTEGRATION_TIME_DEFAULT = TCS34725_INTEGRATIONTIME_50MS;
	public final static int GAIN_DEFAULT = TCS34725_GAIN_4X;
	private boolean verbose = false;
	private TCSColor tcsColor = new TCSColor();
	private boolean initialized = false;

	private int integrationTime;
	private int gain;
	private ThreadRateControl trc = new ThreadRateControl();
	private TimeoutTimer timeoutTimer = new TimeoutTimer(INTEGRATION_TIME_DELAY.get(INTEGRATION_TIME_DEFAULT));
	private byte[] readBuffer = new byte[2];

	private Notifier notifier = new Notifier(() -> {
		if (!initialized) {
			try {
				initialize(integrationTime, gain);
			} catch (Exception ex) {
				setInitialized(false);
			}
		}

		tcsColor.setColor();
	});

	/**
	 * Constructs the TCS34725 RGB color sensor over onboard I2C port.
	 *
	 * @param verbose   If true, spew helpful messages to console. If omitted, assume false.
	 */
	public TCS34725(boolean... verbose) {
		this(I2C.Port.kOnboard, verbose);
	}

	/**
	 * Constructs the TCS34725 RGB color sensor over specified I2C port using
	 * INTEGRATION_TIME_DEFAULT and GAIN_DEFAULT settings.
	 *
	 * @param port      The I2C port the sensor is attached to
	 * @param verbose   If true, spew helpful messages to console. If omitted, assume false.
	 */
	public TCS34725(I2C.Port port, boolean... verbose) {
		this(port, INTEGRATION_TIME_DEFAULT, GAIN_DEFAULT, verbose);
	}

	/**
	 * Constructs the TCS34725 RGB color sensor over specified I2C port with a specified
	 * integration time and gain.
	 *
	 * @param port				The I2C port the sensor is attached to
	 * @param integrationTime	Set the about of time to sense per sample. Integration Time = 2.4 ms × (256 − integrationTime).
	 * 							TCS34725_INTEGRATIONTIME constants available for convenience.
	 * @param gain				Two bit value defining gain from 1X-60X.  See TCS34725_GAIN constants.
	 * @param verbose			If true, spew helpful messages to console. If omitted, assume false.
	 */
	public TCS34725(I2C.Port port, int integrationTime, int gain, boolean... verbose) {
		this(new I2C(port, TCS34725_ADDRESS), integrationTime, gain, verbose);
	}

	/**
	 * Constructs the TCS34725 RGB color sensor over an already instantiated I2C object.
	 *
	 * @param i2c		Initialized I2C object
	 * @param verbose	If true, spew helpful messages to console. If omitted, assume false.
	 */
	public TCS34725(I2C i2c, boolean... verbose) {
		this(i2c, INTEGRATION_TIME_DEFAULT, GAIN_DEFAULT, verbose);
	}

	/**
	 *
	 * @param i2c				Initialized I2C object
	 * @param integrationTime	Set the about of time to sense per sample. Integration Time = 2.4 ms × (256 − integrationTime).
	 * 							TCS34725_INTEGRATIONTIME constants available for convenience.
	 * @param gain				Two bit value defining gain from 1X-60X.  See TCS34725_GAIN constants.
	 * @param verbose			If true, spew helpful messages to console. If omitted, assume false.
	 */
	public TCS34725(I2C i2c, int integrationTime, int gain, boolean... verbose) {
		HAL.report(FRCNetComm.tResourceType.kResourceType_I2C, kDeviceID, 0, "TCS34725_ColorSensor");
		if (i2c == null) {
			throw new IllegalArgumentException("i2c cannot be null");
		}
		this.i2c = i2c;
		this.verbose = verbose.length > 0 ? verbose[0] : false;
		try {
			initialize(integrationTime, gain);
		} catch (Exception e) {
			System.err.println(e.getMessage());
			throw new RuntimeException(e);
		} catch (Error err) {
			throw new RuntimeException(err);
		}
	}

	/**
	 * Verbosity will write trace information to the console.
	 *
	 * @param b	Set to true to enable
	 */
	private void setVerbose(boolean b) {
		this.verbose = b;
	}

	/**
	 * Check to make sure the I2C device we are expecting to be on the bus actually is.
	 * Then, enable it.
	 *
	 * @throws Exception
	 */
	private void initialize(int integrationTime, int gain) throws TransferAbortedException {
		trc.start();
		timeoutTimer.reset();
		if (gain > TCS34725_GAIN_60X | gain < 0) {
			throw new IllegalArgumentException("Gain not valid.");
		}
		int result = this.readU8(TCS34725_ID);
		if (result != kDeviceID) {
			throw new RuntimeException("Device is not a TCS34721/TCS34725");
		}
		// Set startup integration time and gain
		setIntegrationTime(integrationTime);
		setGain(gain);
		// Power on
		enable();
		setInitialized(true);
		if (verbose) {
			System.out.println("TCS34725 initialized");
		}
	}

	private synchronized void setInitialized(boolean initialized) {
		this.initialized = initialized;
	}


	/**
	 * Enable color sensor sensing.
	 *
	 * @throws TransferAbortedException
	 */
	private void enable() throws TransferAbortedException {
		this.write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
		trc.start();
		trc.doRateControl(10); // Per datasheet, at least 2.4ms must elapse before AEN can be asserted
		this.write8(TCS34725_ENABLE, (TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
		timeoutTimer.reset();
		if (verbose) {
			System.out.println("TCS34725 enabled");
		}
	}

	public boolean isEnabled() throws TransferAbortedException {
		int reg = 0;
		reg = this.readU8(TCS34725_ENABLE);
		return ((reg & TCS34725_ENABLE_PON) != 0 && (reg & TCS34725_ENABLE_AEN) != 0);
	}

	private void disable() throws TransferAbortedException {
		// Datasheet does not say it explicitly, but you must wait the 2.4ms between
		// turning off AEN and PON in order to get the device into the sleep state.
		int reg = 0;
		reg = this.readU8(TCS34725_ENABLE);
		this.write8(TCS34725_ENABLE, (reg & ~(TCS34725_ENABLE_AEN)));
		trc.start();
		trc.doRateControl(10);
		this.write8(TCS34725_ENABLE, (reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)));
	}

	private void setIntegrationTime(int integrationTime) throws TransferAbortedException {
		this.integrationTime = integrationTime;
		timeoutTimer = new TimeoutTimer(INTEGRATION_TIME_DELAY.get(integrationTime));
		this.write8(TCS34725_ATIME, integrationTime);
	}

	private int getIntegrationTime() throws TransferAbortedException {
		return this.readU8(TCS34725_ATIME);
	}

	private void setGain(int gain) throws TransferAbortedException {
		this.gain = gain;
		this.write8(TCS34725_CONTROL, gain);
	}

	private int getGain() throws TransferAbortedException {
		return this.readU8(TCS34725_CONTROL);
	}

	private TCSColor prevTCSColor = new TCSColor(0, 0, 0, 0);
	private TCSColor getRawData() {
		if (timeoutTimer.isTimedOut()) {
			int r = 0;
			int g = 0;
			int b = 0;
			int c = 0;
			try {
				r = this.readU16(TCS34725_RDATAL);
				g = this.readU16(TCS34725_GDATAL);
				b = this.readU16(TCS34725_BDATAL);
				c = this.readU16(TCS34725_CDATAL);
			} catch (Exception ex) {
				r = 0;
				g = 0;
				b = 0;
				c = 0;
			} finally {
				// Reset the integration time future so that polling this method causes enough time
				// to elapse between samples.
				timeoutTimer.reset();
			}
			prevTCSColor.setColor(r, g, b, c);
		}
		return prevTCSColor;
	}

	private void setInterrupt(boolean intrpt) throws Exception {
		int r = this.readU8(TCS34725_ENABLE);
		if (intrpt) {
			r |= TCS34725_ENABLE_AIEN;
		} else {
			r &= ~TCS34725_ENABLE_AIEN;
		}
		this.write8(TCS34725_ENABLE, r);
	}

	private int getColorTemp() {
		return calculateColorTemperature(getRawData());
	}

	private ColorOutput getColorOutput() {
		return getRawData().getColorOutput();
	}

	/**
	 * Converts the raw R/G/B values to color temperature in degrees Kelvin
	 * see http://en.wikipedia.org/wiki/Color_temperature
	 */
	private static int calculateColorTemperature(TCSColor rgb) {
		// 1. Map RGB values to their XYZ counterparts.
		// Based on 6500K fluorescent, 3000K fluorescent
		// and 60W incandescent values for a wide range.
		// Note: Y = Illuminance or lux
		double X = (-0.14282 * rgb.getR()) + (1.54924 * rgb.getG()) + (-0.95641 * rgb.getB());
		double Y = (-0.32466 * rgb.getR()) + (1.57837 * rgb.getG()) + (-0.73191 * rgb.getB());
		double Z = (-0.68202 * rgb.getR()) + (0.77073 * rgb.getG()) + (0.56332 * rgb.getB());

		// 2. Calculate the chromaticity co-ordinates
		double xc = (X) / (X + Y + Z);
		double yc = (Y) / (X + Y + Z);

		// 3. Use McCamy's formula to determine the CCT
		double n = (xc - 0.3320) / (0.1858 - yc);

		// Calculate the final CCT
		double cct = (449.0 * Math.pow(n, 3.0)) + (3525.0 * Math.pow(n, 2.0)) + (6823.3 * n) + 5520.33;

		return (int) cct;
	}

	/**
	 * Values in Lux (or Lumens) per square meter.
	 */
	private static int calculateLux(TCSColor rgb) {
		double illuminance = (-0.32466 * rgb.getR()) + (1.57837 * rgb.getG()) + (-0.73191 * rgb.getB());
		return (int) illuminance;
	}

	/**
	 * Write a byte to a register
	 *
	 * @param register	The register to write to
	 * @param value		The value to write
	 * @throws TransferAbortedException
	 */
	private void write8(int register, int value) throws TransferAbortedException {
		if (i2c.write(TCS34725_COMMAND_BIT | register, (byte) (value & 0xff)) == true) {
			throw new TransferAbortedException("Write aborted");
		}
		if (verbose) {
			System.out.println("(U8) I2C: Device " + toHex(TCS34725_ADDRESS) + " wrote " + toHex(value) + " to reg " + toHex(~TCS34725_COMMAND_BIT & register));
		}
	}

	/**
	 * Read an unsigned 16-bit interger
	 *
	 * @param register	Register to read from
	 * @return 			Value read
	 * @throws TransferAbortedException
	 */
	private int readU16(int register) throws TransferAbortedException {
		readBuffer[0] = 0;
		readBuffer[1] = 0;
		if (i2c.read(TCS34725_COMMAND_BIT | TCS34725_COMMAND_AUTO_INCREMENT | register, 2, readBuffer) == true) {
			throw new TransferAbortedException("Read aborted");
		}

		int result = ((readBuffer[1] & 0xFF) << 8) | (readBuffer[0] & 0xFF);
		if (verbose) {
			System.out.println("(U16) I2C: Device " + toHex(TCS34725_ADDRESS) + " returned " + toHex(result) + " from reg " + toHex(~TCS34725_COMMAND_BIT & register));
		}
		return result;
	}

	/**
	 * Read an unsigned byte from the I2C device
	 *
	 * @param reg	Register to read from
	 * @return		Result read
	 * @throws TransferAbortedException
	 */
	private int readU8(int reg) throws TransferAbortedException {
		int result = 0;
		readBuffer[0] = 0;
		readBuffer[1] = 0;
		if (i2c.read(TCS34725_COMMAND_BIT | reg, 1, readBuffer) == true) {
			throw new TransferAbortedException("Read aborted");
		}
		result = readBuffer[0] & 0xFF;
		if (verbose) {
			System.out.println("(U8) I2C: Device " + toHex(TCS34725_ADDRESS) + " returned " + toHex(result) + " from reg " + toHex(~TCS34725_COMMAND_BIT & reg));
		}
		return result;
	}

	private static String toHex(int i) {
		String s = Integer.toString(i, 16).toUpperCase();
		while (s.length() % 2 != 0) {
			s = "0" + s;
		}
		return "0x" + s;
	}

	private static class TransferAbortedException extends Exception {
		public TransferAbortedException(String message) {
			super(message);
		}
	}

	private static class TCSColor {
		private int r, g, b, c;
		private int h, s, v;
		private double cCMYK, m, y, k;
		float[] hsv = new float[3];
		float[] cmyk = new float[4];

		public TCSColor() {
			setColor(0, 0, 0, 0);
		}

		public TCSColor(TCSColor tcsColor) {
			setColor(tcsColor.r, tcsColor.g, tcsColor.b, tcsColor.c);
		}

		public TCSColor(int r, int g, int b, int c) {
			setColor(r, g, b, c);
		}

		public synchronized void setColor(int r, int g, int b, int c) {
			this.r = r;
			this.g = g;
			this.b = b;
			this.c = c;

			// RGBtoHSV(r,g,b,hsv);
			// h = (int)(hsv[0] * 360.0);
			// s = (int)(hsv[1] * 255.0);
			// v = (int)(hsv[2] * 255.0);

			RGBtoCMYK(r, g, b, cmyk);
			cCMYK = cmyk[0];
			m = cmyk[1];
			y = cmyk[2];
			k = cmyk[3];
		}

		public synchronized void setColor(TCSColor tcsColor) {
			this.r = r;
			this.g = g;
			this.b = b;
			this.c = c;

			// RGBtoHSV(r,g,b,hsv);
			// h = (int)(hsv[0] * 360.0);
			// s = (int)(hsv[1] * 255.0);
			// v = (int)(hsv[2] * 255.0);

			RGBtoCMYK(r, g, b, cmyk);
			cCMYK = cmyk[0];
			m = cmyk[1];
			y = cmyk[2];
			k = cmyk[3];
		}

		public int getR() {
			return r;
		}

		public int getG() {
			return g;
		}

		public int getB() {
			return b;
		}

		public int getC() {
			return c;
		}

		public int getH() {
			return h;
		}

		public int getS() {
			return s;
		}

		public int getV() {
			return v;
		}

		public synchronized ColorOutput getColorOutput() {
			return getColorFromCMYKValue();
		}

		public String toString() {
			return "[ r:" + r +
					", g:" + g +
					", b:" + b +
					", h:" + h +
					", s:" + s +
					", v:" + v +
					", cCMYK:" + cCMYK +
					", m:" + m +
					", y:" + y +
					", k:" + k +
					", c:" + c + "]";
		}

		public synchronized ColorOutput getColorFromCMYKValue() {
			if (cCMYK >= 85 && m <= 50 && y < 20) {
				return ColorOutput.BLUE;
			} else if (cCMYK >= 20 && m < 20 && y >= 50) {
				return ColorOutput.GREEN;
			} else if (cCMYK < 5 && m >= 75 && y >= 85) {
				return ColorOutput.RED;
			} else if (cCMYK < 5 && m < 60 && y >= 85) {
				return ColorOutput.YELLOW;
			}
	
			return ColorOutput.NONE;
		}

		public synchronized ColorOutput getColorFromHueValue() {
			if (h >= 330 || h <= 12) {
				return ColorOutput.RED;
			}
			else if (h >= 65 && h <= 160) {
				return ColorOutput.GREEN;
			}
			else if (h >= 165 && h <= 285) {
				return ColorOutput.BLUE;
			}
			else if (h >= 20 && h <= 55) {
				return ColorOutput.YELLOW;
			}
	
			return ColorOutput.NONE;
		}

		private synchronized static float[] RGBtoCMYK(int r, int g, int b, float[] cmyk) {
			int cmax = (r > g) ? r : g;
			if (b > cmax) cmax = b;
			int cmin = (r < g) ? r : g;
			if (b < cmin) cmin = b;
			
			if (cmax != 0) {
				float redc = ((float) (cmax - r)) / ((float) (cmax - cmin));
				float greenc = ((float) (cmax - g)) / ((float) (cmax - cmin));
				float bluec = ((float) (cmax - b)) / ((float) (cmax - cmin));
				
				if (r == cmax) {
					redc = 255.0f;
					greenc = (1.0f - greenc) * 255.0f;
					bluec = (1.0f - bluec) * 255.0f;
				} else if (g == cmax) {
					greenc = 255.0f;
					redc = (1.0f - redc) * 255.0f;
					bluec = (1.0f - bluec) * 255.0f;
				} else if (b == cmax) {
					bluec = 255.0f;
					redc = (1.0f - redc) * 255.0f;
					greenc = (1.0f - greenc) * 255.0f;
				}

				// System.out.println("Redc:"+redc+",Greenc:"+greenc+",Bluec:"+bluec);

				float max = (Math.max(Math.max(redc, greenc), bluec));
				float K = 1 - max;
				float C = (1 - redc - K) / (1 - K);
				float M = (1 - greenc - K) / (1 - K);
				float Y = (1 - bluec - K) / (1 - K);
				cmyk[0] = C * 100;
				cmyk[1] = M * 100;
				cmyk[2] = Y * 100;
				cmyk[3] = K * 100;
			} else {
				cmyk[0] = 0;
				cmyk[1] = 0;
				cmyk[2] = 0;
				cmyk[3] = 0;
			}
			return cmyk;
		}

		public synchronized static float[] RGBtoHSV(int r, int g, int b, float[] hsvVals) {
			float hue, saturation, brightness;
			if (hsvVals == null) {
				hsvVals = new float[3];
			}
			int cmax = (r > g) ? r : g;
			if (b > cmax) cmax = b;
			int cmin = (r < g) ? r : g;
			if (b < cmin) cmin = b;

			brightness = ((float) cmax) / 255.0f;
			if (cmax != 0)
				saturation = ((float) (cmax - cmin)) / ((float) cmax);
			else
				saturation = 0;
			if (saturation == 0)
				hue = 0;
			else {
				float redc = ((float) (cmax - r)) / ((float) (cmax - cmin));
				float greenc = ((float) (cmax - g)) / ((float) (cmax - cmin));
				float bluec = ((float) (cmax - b)) / ((float) (cmax - cmin));
				if (r == cmax)
					hue = bluec - greenc;
				else if (g == cmax)
					hue = 2.0f + redc - bluec;
				else
					hue = 4.0f + greenc - redc;
				hue = hue / 6.0f;
				if (hue < 0)
					hue = hue + 1.0f;
			}
			hsvVals[0] = hue;
			hsvVals[1] = saturation;
			hsvVals[2] = brightness;
			return hsvVals;
		}
	}
}