package com.team195.frc;

import com.team195.lib.drivers.CKAddressableLEDBuffer;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

public class LEDNeoPixelTest {
	@Test
	public void privateOverrideTest() {
		CKAddressableLEDBuffer ledBuffer = new CKAddressableLEDBuffer(20, 0.2f, 0.99f);
		assertNotEquals(ledBuffer.getRawBuffer(), null);
	}

	@Test
	public void ledFadeTest() {
		CKAddressableLEDBuffer ledBuffer = new CKAddressableLEDBuffer(20, 0.2f, 0.99f);
		ledBuffer.flood(210, 0, 120);
		for (int i = 0; i < 200; i++) {
			ledBuffer.stepFade();
			System.out.println(String.format("0x%08X", ledBuffer.getRGB(0)));
		}
	}
}
