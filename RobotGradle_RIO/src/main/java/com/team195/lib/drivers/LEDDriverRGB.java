package com.team195.lib.drivers;

import com.team195.lib.util.RGBColor;
import edu.wpi.first.wpilibj.DigitalOutput;

import java.awt.*;

public class LEDDriverRGB implements LEDDriver{
	private static final int PWM_FREQ = 500;

	private DigitalOutput rLED;
	private DigitalOutput gLED;
	private DigitalOutput bLED;

	private int redPWMOut = 255;
	private int greenPWMOut = 255;
	private int bluePWMOut = 255;

	private float mPrevInc = 0;
	private float[] mCurrColorHSV = new float[3];
	private int mCurrColorRGB = 0;

	private final float kMinBrightness = 0.35f;
	private final float kMaxBrightness = 0.9f;
	private final float kFadeIncrement = 0.032f;

	private boolean on = false;

	public LEDDriverRGB(DigitalOutput rLED, DigitalOutput gLED, DigitalOutput bLED) {
		this.rLED = rLED;
		this.gLED = gLED;
		this.bLED = bLED;

		this.rLED.setPWMRate(PWM_FREQ);
		this.gLED.setPWMRate(PWM_FREQ);
		this.bLED.setPWMRate(PWM_FREQ);
		this.rLED.enablePWM(0);
		this.gLED.enablePWM(0);
		this.bLED.enablePWM(0);
	}

	public synchronized void set(boolean on) {
		if (on) {
			rLED.updateDutyCycle(redPWMOut/255.0);
			gLED.updateDutyCycle(greenPWMOut/255.0);
			bLED.updateDutyCycle(bluePWMOut/255.0);
		} else {
			rLED.updateDutyCycle(0);
			gLED.updateDutyCycle(0);
			bLED.updateDutyCycle(0);
		}
		this.on = on;
	}

	public synchronized void setLEDColor(RGBColor rgbColor) {
		setLEDColor(rgbColor.red, rgbColor.green, rgbColor.blue);
	}

	@SuppressWarnings("DuplicatedCode")
	@Override
	public void processFade() {
		float incDec = 0;

		if (mCurrColorHSV[2] < kMaxBrightness && mPrevInc >= 0.0f) {
			incDec = kFadeIncrement;
		} else if (mPrevInc <= 0.0f && mCurrColorHSV[2] > kMinBrightness) {
			incDec = -kFadeIncrement;
		}

		mCurrColorHSV[2] += incDec;
		mCurrColorHSV[2] = Math.max(Math.min(mCurrColorHSV[2], kMaxBrightness), kMinBrightness);
		mPrevInc = incDec;

		mCurrColorRGB = Color.HSBtoRGB(mCurrColorHSV[0], mCurrColorHSV[1], mCurrColorHSV[2]);
		setLEDColorUpdate((mCurrColorRGB >> 16) & 0xFF, (mCurrColorRGB >> 8) & 0xFF, mCurrColorRGB & 0xFF);
	}

	@Override
	public void processFadeWithSyncPixel(FloatingPixel f, int pixelRateDivisor, boolean forward, boolean startPixelWhenDim) {
		//Not supported
		processFade();
	}

	public synchronized void setLEDColor(int redPWMOut, int greenPWMOut, int bluePWMOut) {
		setLEDColorUpdate(redPWMOut, greenPWMOut, bluePWMOut);
		set(on);
	}

	private synchronized  void setLEDColorUpdate(int redPWMOut, int greenPWMOut, int bluePWMOut) {
		this.redPWMOut = redPWMOut;
		this.greenPWMOut = greenPWMOut;
		this.bluePWMOut = bluePWMOut;
	}

}
