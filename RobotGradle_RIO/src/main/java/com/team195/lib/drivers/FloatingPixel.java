package com.team195.lib.drivers;

import com.team195.lib.util.RGBColor;

public class FloatingPixel {
	public int index = 0;
	public final int mStripLength;
	public RGBColor mRGBColor;

	public FloatingPixel(RGBColor rgbColor, int stripLength) {
		mRGBColor = rgbColor;
		mStripLength = stripLength;
	}

	public boolean isAtEnd() {
		return index >= mStripLength || index < 0;
	}

	public void increment() {
		index++;
	}

	public void decrement() {
		index--;
	}

	public void reset() {
		if (index >= mStripLength) {
			index = 0;
		}
		else if (index < 0) {
			index = mStripLength - 1;
		}
	}
}