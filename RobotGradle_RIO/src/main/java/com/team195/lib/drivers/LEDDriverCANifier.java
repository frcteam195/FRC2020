package com.team195.lib.drivers;

import com.ctre.phoenix.CANifier;
import com.team195.frc.constants.Constants;
import com.team195.lib.util.RGBColor;

import java.awt.*;

public class LEDDriverCANifier implements LEDDriver {
	private CANifier canifier;
	private RGBColor rgbColor = Constants.kDefaultColor;
	private RGBColor mPrevRGBColor = new RGBColor(0,0,0);
	private float mPrevInc = 0;
	private float[] mCurrColorHSV = new float[3];
	private int mCurrColorRGB = 0;
	private boolean on = false;

	private final float kMinBrightness = 0.35f;
	private final float kMaxBrightness = 0.9f;
	private final float kFadeIncrement = 0.032f;

	public LEDDriverCANifier(CANifier canifier) {
		this.canifier = canifier;
		set(false);
	}

	@Override
	public synchronized void set(boolean on) {
		if (this.on != on || !rgbColor.equals(mPrevRGBColor)) {

			if (on) {
				canifier.setLEDOutput(rgbColor.red / 255.0, CANifier.LEDChannel.LEDChannelA);
				canifier.setLEDOutput(rgbColor.green / 255.0, CANifier.LEDChannel.LEDChannelB);
				canifier.setLEDOutput(rgbColor.blue / 255.0, CANifier.LEDChannel.LEDChannelC);
			} else {
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelA);
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelB);
				canifier.setLEDOutput(0, CANifier.LEDChannel.LEDChannelC);
			}

			mPrevRGBColor = rgbColor;
			this.on = on;
		}
	}

	@Override
	public synchronized void setLEDColor(RGBColor rgbColor) {
		Color.RGBtoHSB(rgbColor.red, rgbColor.green, rgbColor.blue, mCurrColorHSV);
		setLEDColorUpdate(rgbColor);
	}

	private synchronized void setLEDColorUpdate(RGBColor rgbColor) {
		this.rgbColor = rgbColor;
		set(on);
	}

	@SuppressWarnings("DuplicatedCode")
	@Override
	public synchronized void processFade() {
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
		setLEDColorUpdate(new RGBColor((mCurrColorRGB >> 16) & 0xFF, (mCurrColorRGB >> 8) & 0xFF, mCurrColorRGB & 0xFF));
	}

	@Override
	public void processFadeWithSyncPixel(FloatingPixel f, int pixelRateDivisor, boolean forward, boolean startPixelWhenDim) {
		//Not supported
	}
}
