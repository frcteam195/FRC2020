package com.team195.lib.drivers;

import com.team195.frc.constants.Constants;
import com.team195.lib.util.RGBColor;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDDriverNeoPixel implements LEDDriver {
	private final AddressableLED mLED;
	private final CKAddressableLEDBuffer mNeoPixelBuffer;
	private final CKAddressableLEDBuffer mNeoPixelOffBuffer;


	private boolean on = false;

	public LEDDriverNeoPixel(AddressableLED led, int ledCount) {
		mLED = led;
		mNeoPixelBuffer = new CKAddressableLEDBuffer(ledCount, 0.35f, 0.9f, 0.032f);
		mNeoPixelOffBuffer = new CKAddressableLEDBuffer(ledCount);
		mLED.setLength(mNeoPixelBuffer.getLength());
		mLED.start();
	}

	@Override
	public synchronized void set(boolean on) {
		if (on) {
			mLED.setData(mNeoPixelBuffer);
		} else {
			mLED.setData(mNeoPixelOffBuffer);
		}
		this.on = on;
	}

//	public synchronized int getLength() {
//		return mNeoPixelBuffer.getLength();
//	}

	@Override
	public synchronized void processFade() {
		mNeoPixelBuffer.stepFade();
	}

	@Override
	public synchronized void processFadeWithSyncPixel(FloatingPixel f, int pixelRateDivisor, boolean forward, boolean startPixelWhenDim) {
		mNeoPixelBuffer.stepFadeWithSyncPixel(f, pixelRateDivisor, forward, startPixelWhenDim);
	}

	public synchronized void floatPixel(FloatingPixel f) {
		mNeoPixelBuffer.processFloatingPixel(f);
	}

	public synchronized void floatPixelFwd(FloatingPixel f) {
		mNeoPixelBuffer.processFloatingPixelForward(f);
	}

	public synchronized void floatPixelBkwd(FloatingPixel f) {
		mNeoPixelBuffer.processFloatingPixelBackward(f);
	}

	@Override
	public synchronized void setLEDColor(RGBColor rgbColor) {
		setLEDColor(rgbColor.red, rgbColor.green, rgbColor.blue);
	}

	public synchronized void setLEDColor(int r, int g, int b) {
		mNeoPixelBuffer.flood(r, g, b);
	}

}