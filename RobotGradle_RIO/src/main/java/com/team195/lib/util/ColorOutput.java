package com.team195.lib.util;

public enum ColorOutput {
	RED,
	GREEN,
	BLUE,
	YELLOW,
	NONE;

	public static ColorOutput fromValue(int hue) {
		if (hue >= 330 || hue <= 12) {
			return RED;
		}
		else if (hue >= 65 && hue <= 160) {
			return GREEN;
		}
		else if (hue >= 165 && hue <= 285) {
			return BLUE;
		}
		else if (hue >= 20 && hue <= 55) {
			return YELLOW;
		}

		return NONE;
	}
}
