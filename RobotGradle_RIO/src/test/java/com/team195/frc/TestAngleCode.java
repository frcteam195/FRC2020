package com.team195.frc;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;

public class TestAngleCode {
	private Rotation2d horizontalPlaneToLens = Rotation2d.fromDegrees(15.17);

	public static final double kHorizontalFOV = 59.6; // degrees
	public static final double kVerticalFOV = 49.7; // degrees
	public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
	public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
	public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

	@Test
	public void testTwoVisionCodes() {

	}

	public Translation2d doRayPlane(double tx, double ty) {
		List<double[]> corners = Arrays.asList(new double[] {0, 0, 0, 0});
		if (corners == null) {
			return null;
		}

//		double slope = 1.0;
//		if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
//			slope = (corners.get(1)[1] - corners.get(0)[1]) /
//					(corners.get(1)[0] - corners.get(0)[0]);
//		}

		double x1 = 1.0;
		double y1 = 0;
		double z1 = 0;

		for (int i = 0; i < 2; ++i) {
			// Average of y and z;
			double y_pixels = corners.get(i)[0];
			double z_pixels = corners.get(i)[1];

			// Redefine to robot frame of reference.
			double nY = -((y_pixels - 160.0) / 160.0);
			double nZ = -((z_pixels - 120.0) / 120.0);

			y1 = kVPW / 2 * nY;
			z1 = kVPH / 2 * nZ;

//			target.setSkew(slope);
		}

		// Compensate for camera pitch
//		Translation2d xz_plane_translation = new Translation2d(x1, z1).rotateBy(horizontalPlaneToLens);
//		double x = xz_plane_translation.x();
//		double y = target.getY();
//		double z = xz_plane_translation.y();
//
//		// find intersection with the goal
//		double differential_height = source.getLensHeight() - (high ? Constants.kPortTargetHeight : Constants.kHatchTargetHeight);
//		if ((z < 0.0) == (differential_height > 0.0)) {
//			double scaling = differential_height / -z;
//			double distance = Math.hypot(x, y) * scaling;
//			Rotation2d angle = new Rotation2d(x, y, true);
//			return new Translation2d(distance * angle.cos(), distance * angle.sin());
//		}
//
		return null;
	}
}
