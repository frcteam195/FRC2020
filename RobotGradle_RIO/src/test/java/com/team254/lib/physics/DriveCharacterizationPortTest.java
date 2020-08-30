package com.team254.lib.physics;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertEquals;

public class DriveCharacterizationPortTest {
	public static final double kTestEpsilon = 1e-4;

	@Test
	public void test() {
		final double ks = 6; //Math.random();
		final double kv = 3.3; //Math.random();
		final double ka = 0.002149321267; //Math.random()

		List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
		velocityData.add(new DriveCharacterization.VelocityDataPoint(1, 10));
		velocityData.add(new DriveCharacterization.VelocityDataPoint(2, 12));
		velocityData.add(new DriveCharacterization.VelocityDataPoint(3, 15));
		velocityData.add(new DriveCharacterization.VelocityDataPoint(4, 20));

		List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();
		accelerationData.add(new DriveCharacterization.AccelerationDataPoint(1, 10, 20));
		accelerationData.add(new DriveCharacterization.AccelerationDataPoint(2, 12, 60));
		accelerationData.add(new DriveCharacterization.AccelerationDataPoint(3, 15, 120));
		accelerationData.add(new DriveCharacterization.AccelerationDataPoint(4, 20, 400));


		DriveCharacterization.CharacterizationConstants driveConstants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);
		System.out.println(driveConstants);

		assertEquals(driveConstants.ks, ks, kTestEpsilon);
		assertEquals(driveConstants.kv, kv, kTestEpsilon);
		assertEquals(driveConstants.ka, ka, kTestEpsilon);
	}
}
