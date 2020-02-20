package frc.robot;

import org.junit.Test;

public class TestColor {
	@Test
	public void testCMYKProfile() {
		float[] retVal = TCS34725.rgbToProfiledCmyk(255, 0, 0);
		for (int i = 0; i < retVal.length; i++) {
			System.out.print(retVal + ", ");
		}
		System.out.println();
	}
}
