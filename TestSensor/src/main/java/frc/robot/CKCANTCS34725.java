package frc.robot;

import edu.wpi.first.hal.*;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;

public class CKCANTCS34725 {
	private static final int kPeriodMs = 10;
	private static final int kCANTimeoutMs = 10;

	public static final int CKCANTCS34725_API_NONE = 0x000;
	public static final int CKCANTCS34725_API_SET_PERIODIC_RATE = 0x001;
	public static final int CKCANTCS34725_API_COLOR_SENSOR_DATA = 0x002;

	private final CAN mCANHandle;
	private final byte[] mWriteBuffer = new byte[8];
	private final byte[] mTmp32bArr = new byte[4];

	private float[] mCMYKBuffer = new float[4];
	private float[] mRGBBuffer = new float[3];
	private float[] mHSVBuffer = new float[3];
	private ColorOutput mCachedColor = ColorOutput.NONE;

	private final CANData mCANData = new CANData();

	private final Notifier mNotifier;

	private synchronized void packFloat(float f) {
		mTmp32bArr[0] = (byte)(((int)f & 0xFF000000) >> 24);
		mTmp32bArr[1] = (byte)(((int)f & 0x00FF0000) >> 16);
		mTmp32bArr[2] = (byte)(((int)f & 0x0000FF00) >> 8);
		mTmp32bArr[3] = (byte)(((int)f & 0x000000FF));
	}

	private synchronized void packInt(int i) {
		mTmp32bArr[0] = (byte)((i & 0xFF000000) >> 24);
		mTmp32bArr[1] = (byte)((i & 0x00FF0000) >> 16);
		mTmp32bArr[2] = (byte)((i & 0x0000FF00) >> 8);
		mTmp32bArr[3] = (byte)((i & 0x000000FF));
	}

	private synchronized void clearWriteBuffer() {
		for (int i = 0; i < mWriteBuffer.length; i++) {
			mWriteBuffer[i] = 0;
		}
	}

	private void arrayPrint(byte[] arr) {
		System.out.print("Array: ");
		for (int i = 0; i < arr.length; i++) {
			System.out.print((arr[i] & 0xFF) + ", ");
		}
		System.out.println();
	}

	private synchronized void setCachedColor(ColorOutput color) {
		mCachedColor = color;
	}

	public ColorOutput getColor() {
		return mCachedColor;
	}

	public float[] getmRGBBuffer() { return mRGBBuffer; }

	public float[] getmCMYKBuffer() {
		return mCMYKBuffer;
	}

	public float[] getmHSVBuffer() {
		return mHSVBuffer;
	}

	public CKCANTCS34725(int deviceID) {
		mCANHandle = new CAN(deviceID & 0x3F);

		System.out.println("Initializing CAN Sensor...");

		mNotifier = new Notifier(() -> {
			if (mCANHandle.readPacketLatest(2, mCANData)) {
				mRGBBuffer[0] = mCANData.data[5];
				mRGBBuffer[1] = mCANData.data[6];
				mRGBBuffer[2] = mCANData.data[7];
				RGBtoCMYK(mRGBBuffer, mCMYKBuffer);
				RGBtoHSV(mRGBBuffer, mHSVBuffer);
				setCachedColor(getColorFromCoupledOutput(mCMYKBuffer, mHSVBuffer));
//				arrayPrint(mCANData.data);
			} else {
//				System.out.println("CAN Read Error");
			}
		});

		mNotifier.startPeriodic(0.1);
		System.out.println("Initialized CAN Sensor!");
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
}
