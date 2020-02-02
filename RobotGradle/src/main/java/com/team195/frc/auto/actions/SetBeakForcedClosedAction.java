package com.team195.frc.auto.actions;

import com.team195.frc.constants.AutoConstants;
import com.team195.frc.subsystems.Turret;
import com.team195.lib.util.TimeoutTimer;

public class SetBeakForcedClosedAction implements Action {
	private static final Turret mTurret = Turret.getInstance();
	private final TimeoutTimer mTimeoutTimer = new TimeoutTimer(AutoConstants.kDefaultSolenoidWait * 2.0);

	public SetBeakForcedClosedAction() {
	}

	@Override
	public boolean isFinished() {
		return mTimeoutTimer.isTimedOut();
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		mTurret.setBeakFeedOff(false);
		mTurret.setBeak(true);
		mTimeoutTimer.reset();
	}
}
