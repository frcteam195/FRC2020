package com.team195.frc.auto.actions.climb;

import com.team195.frc.auto.actions.Action;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.subsystems.BallIntakeArm;
import com.team195.frc.subsystems.Drive;
import com.team195.lib.util.TimeoutTimer;

public class SetIntakeBarForwardClimbAction implements Action {
	private static final Drive mDrive = Drive.getInstance();
	private static final BallIntakeArm mBallIntakeArm = BallIntakeArm.getInstance();

	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(6);

	public SetIntakeBarForwardClimbAction() {

	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut()
				|| (mDrive.getRoll() > 12);
//				|| (mDrive.getRoll() > 18);
	}

	@Override
	public void update() {
		ConsoleReporter.report("Climbing:");
	}

	@Override
	public void done() {
//		mDrive.setClimbRight(1);
	}

	@Override
	public void start() {
		mDrive.configureClimbCurrentLimit();
		mDrive.setClimbRight(1);
	}
}
