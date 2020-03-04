package frc.robot;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;
import java.lang.reflect.Field;

public class CKAddressableLEDBuffer extends AddressableLEDBuffer {
	private static final float kDefaultMinBrightness = 0.2f;
	private static final float kDefaultMaxBrightness = 0.99f;
	private static final float kDefaultFadeIncrement = 0.0135f;

	private byte[] m_bufferHandle;
	private float mPrevInc = 0;
	private float[] mCurrColorHSV = new float[3];
	private int mCurrColorRGB = 0;
	private final float mMinBrightness;
	private final float mMaxBrightness;

	public CKAddressableLEDBuffer(int length) {
		this(length, kDefaultMinBrightness, kDefaultMaxBrightness);
	}

	/**
	 * Constructs a new LED buffer with the specified length.
	 *
	 * @param length The length of the buffer in pixels
	 */
	public CKAddressableLEDBuffer(int length, float minBrightness, float maxBrightness) {
		super(length);
		mMinBrightness = minBrightness;
		mMaxBrightness = maxBrightness;

		//Extract private buffer from superclass for extra NeoPixel functions
		try {
			Field fieldBuf = this.getClass().getSuperclass().getDeclaredField("m_buffer");
			fieldBuf.setAccessible(true);
			m_bufferHandle = (byte[]) fieldBuf.get(this);
		} catch (Exception ex) {
			System.out.println("Could not modify private LED Buffer Address!");
			m_bufferHandle = null;
		}
	}

	public synchronized void flood(RGBColor rgbColor) {
		flood(rgbColor.red, rgbColor.green, rgbColor.blue);
	}

	public synchronized void flood(int r, int g, int b) {
		floodUpdate(r, g, b);
		Color.RGBtoHSB(r, g, b, mCurrColorHSV);
	}

	private synchronized void floodUpdate(int r, int g, int b) {
		for (int i = 0; i < getLength(); i++) {
			setRGB(i, r, g, b);
		}
	}

	public synchronized void stepFade() {
		float incDec = 0;

		if (mCurrColorHSV[2] < mMaxBrightness && mPrevInc >= 0.0f) {
			incDec = kDefaultFadeIncrement;
		} else if (mPrevInc <= 0.0f && mCurrColorHSV[2] > mMinBrightness) {
			incDec = -kDefaultFadeIncrement;
		}

		mCurrColorHSV[2] += incDec;
		mCurrColorHSV[2] = Math.max(Math.min(mCurrColorHSV[2], 1.0f), -1.0f);
		mPrevInc = incDec;

		mCurrColorRGB = Color.HSBtoRGB(mCurrColorHSV[0], mCurrColorHSV[1], mCurrColorHSV[2]);
		floodUpdate((mCurrColorRGB >> 16) & 0xFF, (mCurrColorRGB >> 8) & 0xFF, mCurrColorRGB & 0xFF);
	}

	public synchronized void processFloatingPixel(FloatingPixel f) {
		if (!f.isAtEnd()) {
			setRGB(f.index, f.mRGBColor.red, f.mRGBColor.green, f.mRGBColor.blue);
		}
	}

	public synchronized void processFloatingPixelForward(FloatingPixel f) {
		if (!f.isAtEnd()) {
			setRGB(f.index, f.mRGBColor.red, f.mRGBColor.green, f.mRGBColor.blue);
			f.increment();
		}
	}

	public synchronized void processFloatingPixelBackward(FloatingPixel f) {
		if (!f.isAtEnd()) {
			setRGB(f.index, f.mRGBColor.red, f.mRGBColor.green, f.mRGBColor.blue);
			f.decrement();
		}
	}

	public int getRGB(int index) {
		int retVal = 0;
		retVal |= (m_bufferHandle[(index * 4) + 2] << 16) & 0xFF0000;
		retVal |= (m_bufferHandle[(index * 4) + 1] << 8) & 0xFF00;
		retVal |= (m_bufferHandle[(index * 4)]) & 0xFF;
		return retVal;
	}

	public byte[] getRawBuffer() {
		return m_bufferHandle;
	}

}
