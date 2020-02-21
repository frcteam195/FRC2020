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
	private short[] mRawBuffer = new short[4];
	private float[] mCMYKBuffer = new float[4];
	private float[] mRGBBuffer = new float[3];
	private float[] mHSVBuffer = new float[3];
	private ColorOutput mCachedColor = ColorOutput.NONE;
	private boolean initialized = false;

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
			getRawData(mRawBuffer);
			getRGB(mRawBuffer, mRGBBuffer);
			RGBtoCMYK(mRGBBuffer, mCMYKBuffer);
			RGBtoHSV(mRGBBuffer, mHSVBuffer);
			setCachedColor(getColorFromCoupledOutput(mCMYKBuffer, mHSVBuffer));
		} catch (Exception ex) {
			setInitialized(false);
			System.err.println(ex.toString());
		}
	});

	public float[] getmRGBBuffer() { return mRGBBuffer; }

	public float[] getmCMYKBuffer() {
		return mCMYKBuffer;
	}

	public float[] getmHSVBuffer() {
		return mHSVBuffer;
	}

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

	private short[] getRawData(short[] rgbc) {
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

	private float[] getRGB(short[] rawData, float[] rgb) {
		float sum = rawData[3];

		if (!Float.isFinite(sum) || sum == 0) {
			rgb[0] = rgb[1] = rgb[2] = 0;
		} else {
			rgb[0] = (float) rawData[0] / sum * 255.0f;
			rgb[1] = (float) rawData[1] / sum * 255.0f;
			rgb[2] = (float) rawData[2] / sum * 255.0f;
		}

		return rgb;
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

	private synchronized short readU16(int register) throws TransferAbortedException {
		readBuffer[0] = 0;
		readBuffer[1] = 0;
		if (i2c.read(TCS34725_COMMAND_BIT | TCS34725_COMMAND_AUTO_INCREMENT | register, 2, readBuffer) == true) {
			throw new TransferAbortedException("Read aborted");
		}

		short result = (short)(((readBuffer[1] & 0xFF) << 8) | (readBuffer[0] & 0xFF));
		if (verbose) {
			System.out.println("(U16) I2C: Device " + toHex(TCS34725_ADDRESS) + " returned " + toHex(result) + " from reg " + toHex(~TCS34725_COMMAND_BIT & register));
		}
		return result;
	}

	private synchronized byte readU8(int register) throws TransferAbortedException {
		byte result = 0;
		readBuffer[0] = 0;
		readBuffer[1] = 0;
		if (i2c.read(TCS34725_COMMAND_BIT | register, 1, readBuffer) == true) {
			throw new TransferAbortedException("Read aborted");
		}
		result = (byte)(readBuffer[0] & 0xFF);
		if (verbose) {
			System.out.println("(U8) I2C: Device " + toHex(TCS34725_ADDRESS) + " returned " + toHex(result) + " from reg " + toHex(~TCS34725_COMMAND_BIT & register));
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
	private static float[] RGBtoCMYK(float[] rgbc, float[] cmyk) {
		float cmax = (rgbc[0] > rgbc[1]) ? rgbc[0] : rgbc[1];
		if (rgbc[2] > cmax) cmax = rgbc[2];
		float cmin = (rgbc[0] < rgbc[1]) ? rgbc[0] : rgbc[1];
		if (rgbc[2] < cmin) cmin = rgbc[2];

		if (cmax != 0) {
			float redc = (cmax - rgbc[0]) / (cmax - cmin);
			float greenc = (cmax - rgbc[1]) / (cmax - cmin);
			float bluec = (cmax - rgbc[2]) / (cmax - cmin);

			if (rgbc[0] == cmax) {
				redc = 255.0f;
				greenc = (1.0f - greenc) * 255.0f;
				bluec = (1.0f - bluec) * 255.0f;
			} else if (rgbc[1] == cmax) {
				greenc = 255.0f;
				redc = (1.0f - redc) * 255.0f;
				bluec = (1.0f - bluec) * 255.0f;
			} else if (rgbc[2] == cmax) {
				bluec = 255.0f;
				redc = (1.0f - redc) * 255.0f;
				greenc = (1.0f - greenc) * 255.0f;
			}

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

	public synchronized static float[] RGBtoHSV(float[] rgbVals, float[] hsvVals) {
		float hue, saturation, brightness;
		if (hsvVals == null) {
			hsvVals = new float[3];
		}
		float cmax = (rgbVals[0] > rgbVals[1]) ? rgbVals[0] : rgbVals[1];
		if (rgbVals[2] > cmax) cmax = rgbVals[2];
		float cmin = (rgbVals[0] < rgbVals[1]) ? rgbVals[0] : rgbVals[1];
		if (rgbVals[2] < cmin) cmin = rgbVals[2];

		brightness = ((float) cmax) / 255.0f;
		if (cmax != 0)
			saturation = ((float) (cmax - cmin)) / ((float) cmax);
		else
			saturation = 0;
		if (saturation == 0)
			hue = 0;
		else {
			float redc = (cmax - rgbVals[0]) / (cmax - cmin);
			float greenc = (cmax - rgbVals[1]) / (cmax - cmin);
			float bluec = (cmax - rgbVals[2]) / (cmax - cmin);
			if (rgbVals[0] == cmax)
				hue = bluec - greenc;
			else if (rgbVals[1] == cmax)
				hue = 2.0f + redc - bluec;
			else
				hue = 4.0f + greenc - redc;
			hue = hue / 6.0f;
			if (hue < 0)
				hue = hue + 1.0f;
		}
		hsvVals[0] = hue * 360;
		hsvVals[1] = saturation * 255;
		hsvVals[2] = brightness;
		return hsvVals;
	}

	private static ColorOutput getColorFromCMYKValue(float c, float m, float y, float k) {
		if ((c >= 85 && m <= 50 && y < 20)) {
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

	private static ColorOutput getColorFromCoupledOutput(float[] cmykArr, float[] hsvArr) {
		if ((cmykArr[0] >= 85 && cmykArr[1] <= 50 && cmykArr[2] < 20) && (hsvArr[0] >= 165 && hsvArr[0] <= 285)) {
			return ColorOutput.BLUE;
		} else if ((cmykArr[0] >= 20 && cmykArr[1] < 20 && cmykArr[2] >= 50) && (hsvArr[0] >= 65 && hsvArr[0] <= 160 && hsvArr[1] < 130)) {
			return ColorOutput.GREEN;
		} else if ((cmykArr[0] < 5 && cmykArr[1] >= 70 && cmykArr[2] >= 85) && (hsvArr[0] >= 330 || hsvArr[0] <= 12)) {
			return ColorOutput.RED;
		} else if ((cmykArr[0] < 5 && cmykArr[1] < 70 && cmykArr[2] >= 85) && (hsvArr[0] >= 20 && hsvArr[0] <= 55 && hsvArr[1] >= 100)) {
			return ColorOutput.YELLOW;
		}

		return ColorOutput.NONE;
	}

	public static ColorOutput getColorFromHueValue(float h, float s, float v) {
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
}