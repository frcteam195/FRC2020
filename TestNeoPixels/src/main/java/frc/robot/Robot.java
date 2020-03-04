package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
	private static final int kNeoPixelCount = 20;
	private static final int kNeoPixelPWMPort = 0;
	private static final RGBColor kDefaultRobotColor = new RGBColor(210, 0, 120);
	private static final RGBColor kGamePieceColor = new RGBColor(255, 127, 0);

	LEDDriverNeoPixel mNeoPixels;
	FloatingPixel mFloatingPixel;

//	TimeoutTimer mPixelUpdate = new TimeoutTimer(0.02);
	ElapsedTimer mLEDTimer = new ElapsedTimer();

	@Override
	public void robotInit() {
		mNeoPixels = new LEDDriverNeoPixel(new AddressableLED(kNeoPixelPWMPort), kNeoPixelCount);
		mFloatingPixel = new FloatingPixel(kGamePieceColor, mNeoPixels.getLength());
		mNeoPixels.setLEDColor(kDefaultRobotColor);
		mNeoPixels.set(true);
//		mPixelUpdate.reset();
	}

	@Override
	public void robotPeriodic() {
		mLEDTimer.start();
//		mNeoPixels.processFade();
//
//		if (mPixelUpdate.isTimedOut()) {
//			mNeoPixels.floatPixelFwd(mYellowPixel);
//			mYellowPixel.reset();
//			mPixelUpdate.reset();
//		} else {
//			mNeoPixels.floatPixel(mYellowPixel);
//		}

		mNeoPixels.processFadeWithSyncPixel(mFloatingPixel, 2, true);
		mNeoPixels.set(true);
		double loopTime = mLEDTimer.hasElapsed();
		System.out.println("Loop Time: " + loopTime);
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

}
