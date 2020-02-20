package frc.robot;

import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Notifier;

import java.awt.color.ColorSpace;
import java.awt.color.ICC_ColorSpace;
import java.awt.color.ICC_Profile;
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
 * Converted to threaded cached read with reinit
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

		ICC_ColorSpace tmpColorSpace;
		try {
			tmpColorSpace = new ICC_ColorSpace(ICC_Profile.getInstance(Filesystem.getDeployDirectory() + "/UncoatedFOGRA29.icc"));
		} catch (Exception e) {
			tmpColorSpace = null;
		}
		iccColorSpace = tmpColorSpace;
	}

	public static final int kDeviceID = 0x44;
	public final static int INTEGRATION_TIME_DEFAULT = TCS34725_INTEGRATIONTIME_50MS;
	public final static int GAIN_DEFAULT = TCS34725_GAIN_4X;
	private boolean verbose = false;
	private int integrationTime;
	private int gain;

	private ThreadRateControl trc = new ThreadRateControl();
	private TimeoutTimer timeoutTimer = new TimeoutTimer(INTEGRATION_TIME_DELAY.get(INTEGRATION_TIME_DEFAULT));
	private byte[] readBuffer = new byte[2];
	private float[] mCMYKBuffer = new float[4];
	private int[] mRGBCBuffer = new int[4];
	private ColorOutput mCachedColor = ColorOutput.NONE;
	private boolean initialized = false;

	private static final ColorSpace iccColorSpace;

	private static final double kNotifierPeriod = 0.020;
	private Notifier mNotifier = new Notifier(() -> {
		if (!initialized) {
			try {
				initialize(integrationTime, gain);
			} catch (Exception ex) {
				setInitialized(false);
			}
		}
		try {
			getRawData(mRGBCBuffer);
			getCMYKColor(mRGBCBuffer, mCMYKBuffer);
			setCachedColor(getColorFromCMYKValue(mCMYKBuffer[0], mCMYKBuffer[1], mCMYKBuffer[2], mCMYKBuffer[3]));
		} catch (Exception ex) {
			setInitialized(false);
			System.err.println(ex.toString());
		}
	});

	public TCS34725(boolean... verbose) {
		this(I2C.Port.kOnboard, verbose);
	}

	public TCS34725(I2C.Port port, boolean... verbose) {
		this(port, INTEGRATION_TIME_DEFAULT, GAIN_DEFAULT, verbose);
	}

	public TCS34725(I2C.Port port, int integrationTime, int gain, boolean... verbose) {
		this(new I2C(port, TCS34725_ADDRESS), integrationTime, gain, verbose);
	}

	public TCS34725(I2C i2c, boolean... verbose) {
		this(i2c, INTEGRATION_TIME_DEFAULT, GAIN_DEFAULT, verbose);
	}

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
			setInitialized(false);
		}

		mNotifier.startPeriodic(kNotifierPeriod);
	}

	private synchronized void setVerbose(boolean b) {
		this.verbose = b;
	}

	private synchronized void setCachedColor(ColorOutput color) {
		mCachedColor = color;
	}

	public ColorOutput getColor() {
		return mCachedColor;
	}

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

	private synchronized void setIntegrationTime(int integrationTime) throws TransferAbortedException {
		this.integrationTime = integrationTime;
		timeoutTimer = new TimeoutTimer(INTEGRATION_TIME_DELAY.get(integrationTime));
		this.write8(TCS34725_ATIME, integrationTime);
	}

	private synchronized void setGain(int gain) throws TransferAbortedException {
		this.gain = gain;
		this.write8(TCS34725_CONTROL, gain);
	}

	private int[] getRawData(int[] rgbc) {
		if (timeoutTimer.isTimedOut()) {
			try {
				rgbc[0] = readU16(TCS34725_RDATAL);
				rgbc[1] = readU16(TCS34725_GDATAL);
				rgbc[2] = readU16(TCS34725_BDATAL);
				rgbc[3] = readU16(TCS34725_CDATAL);
			} catch (Exception ex) {
				rgbc[0] = 0;
				rgbc[1] = 0;
				rgbc[2] = 0;
				rgbc[3] = 0;
			} finally {
				// Reset the integration time future so that polling this method causes enough time
				// to elapse between samples.
				timeoutTimer.reset();
			}
		}
		return rgbc;
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

	private synchronized void write8(int register, int value) throws TransferAbortedException {
		if (i2c.write(TCS34725_COMMAND_BIT | register, (byte) (value & 0xff)) == true) {
			throw new TransferAbortedException("Write aborted");
		}
		if (verbose) {
			System.out.println("(U8) I2C: Device " + toHex(TCS34725_ADDRESS) + " wrote " + toHex(value) + " to reg " + toHex(~TCS34725_COMMAND_BIT & register));
		}
	}

	private synchronized int readU16(int register) throws TransferAbortedException {
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

	private synchronized int readU8(int reg) throws TransferAbortedException {
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

	//This method could possibly be improved by using an ICC Color Profile conversion
	//Use this as a fallback when no ICC profile loaded
	private static float[] RGBtoCMYK(int[] rgbc, float[] cmyk) {
		int r = rgbc [0];
		int g = rgbc [1];
		int b = rgbc [2];
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

	private float[] getCMYKColor(int[] rgbc, float[] cmyk) {
		if (iccColorSpace != null) {
			cmyk = rgbToProfiledCmyk(rgbc[0], rgbc[1], rgbc[2]);
			return cmyk;
		} else {
			return RGBtoCMYK(rgbc, cmyk);
		}
	}

	private float[] rgbToProfiledCmyk(float... rgb) {
		if (iccColorSpace == null) {
			return null;
		}

		if (rgb.length != 3) {
			throw new IllegalArgumentException();
		}

		float[] fromRGB = iccColorSpace.fromRGB(rgb);
		return fromRGB;
	}

	private static ColorOutput getColorFromCMYKValue(float c, float m, float y, float k) {
		if (c >= 85 && m <= 50 && y < 20) {
			return ColorOutput.BLUE;
		} else if (c >= 20 && m < 20 && y >= 50) {
			return ColorOutput.GREEN;
		} else if (c < 5 && m >= 75 && y >= 85) {
			return ColorOutput.RED;
		} else if (c < 5 && m < 60 && y >= 85) {
			return ColorOutput.YELLOW;
		}

		return ColorOutput.NONE;
	}
}