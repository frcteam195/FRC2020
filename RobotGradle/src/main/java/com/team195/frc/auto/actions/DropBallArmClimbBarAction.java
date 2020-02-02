package com.team195.frc.auto.actions;

import com.team195.frc.constants.AutoConstants;
import com.team195.frc.subsystems.BallIntakeArm;
import com.team195.lib.util.TimeoutTimer;

public class DropBallArmClimbBarAction implements Action {
	private static final BallIntakeArm mBallIntakeArm = BallIntakeArm.getInstance();

	private TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait + AutoConstants.kArmFallWait);

	public DropBallArmClimbBarAction() {
	}

	@Override
	public boolean isFinished() {
		if (mTimeoutTimer != null)
			return mTimeoutTimer.isTimedOut();
		else
			return true;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		if (mTimeoutTimer != null)
			mTimeoutTimer.reset();

		if (!mBallIntakeArm.isBallIntakeBarClimbLatched())
			mBallIntakeArm.dropClimbBar();
		else
			mTimeoutTimer = null;
	}
}
