package com.team195.lib.drivers;


import com.team195.lib.util.RGBColor;

public interface LEDDriver {
	void set(boolean on);
	void setLEDColor(RGBColor rgbColor);
	void processFade();
	void processFadeWithSyncPixel(FloatingPixel f, int pixelRateDivisor, boolean forward, boolean startPixelWhenDim);
}
