package com.team195.frc;

import com.illposed.osc.OSCBoundListMessage;
import com.team195.frc.constants.Constants;
import com.team195.frc.loops.ILooper;
import com.team195.frc.loops.Loop;
import com.team195.frc.loops.Looper;
import com.team195.frc.reporters.ConsoleReporter;
import com.team195.frc.reporters.DataReporter;
import com.team195.frc.reporters.MessageLevel;
import com.team195.frc.subsystems.Subsystem;
import com.team195.lib.util.ElapsedTimer;
import com.team195.lib.util.Reportable;
import com.team195.lib.util.TimeoutTimer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {

	private static SubsystemManager instance = null;

	private static final ArrayList<Subsystem> mAllSubsystems = new ArrayList<>();
	private static final List<Loop> mLoops = new ArrayList<>();

	private static final ArrayList<Reportable> mLooperReports = new ArrayList<>();

	private TimeoutTimer mCriticalCheckTimeout = new TimeoutTimer(0.250);
	private TimeoutTimer mLogDataTimeout = new TimeoutTimer(0.250);
	private ElapsedTimer eLoopTimer = new ElapsedTimer();

	private SubsystemManager() {

	}

	public static SubsystemManager getInstance(Subsystem... subsystems) {
		if(instance == null) {
			try {
				instance = new SubsystemManager();
			} catch (Exception ex) {
				ConsoleReporter.report(ex, MessageLevel.DEFCON1);
			}
		}

		if (subsystems != null && subsystems.length > 0)
			mAllSubsystems.addAll(Arrays.asList(subsystems));

		return instance;
	}

	private static final ArrayList<Boolean> diagnosticRetVals = new ArrayList<>();
	public boolean checkSystemsPassDiagnostics() {
		diagnosticRetVals.clear();
		mAllSubsystems.forEach((s) -> diagnosticRetVals.add(s.runDiagnostics()));
		for (boolean b : diagnosticRetVals) {
			if (!b)
				return false;
		}
		return true;
	}

	private static final List<Object> l = new ArrayList<>(60);
	private static final OSCBoundListMessage boundOSCMesage = new OSCBoundListMessage("/LogData", l);
	private void generateReport() {
		l.clear();

		l.add("Enabled");
		l.add(Boolean.toString(DriverStation.getInstance().isEnabled()));

		l.add("Timestamp_Robot");
		l.add(Timer.getFPGATimestamp());

//		l.add("MatchTime");
//		l.add(DriverStation.getInstance().getMatchTime());

		try {
			mAllSubsystems.forEach((s) -> l.addAll(s.generateReport()));
			mLooperReports.forEach((s) -> l.addAll(s.generateReport()));
		} catch (Exception ex) {
			ConsoleReporter.report(ex);
		}

		l.add("OverallLoopTime");
		l.add(eLoopTimer.hasElapsed());
	}

	public void stop() {
		mAllSubsystems.forEach(Subsystem::stop);
	}

	private class EnabledLoop implements Loop {

		@Override
		public void onFirstStart(double timestamp) {
			mLoops.forEach((l) -> l.onFirstStart(timestamp));
		}

		@Override
		public void onStart(double timestamp) {
			mLoops.forEach((l) -> l.onStart(timestamp));
		}

		@Override
		public void onLoop(double timestamp) {
			eLoopTimer.start();
			mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
			mLoops.forEach((l) -> l.onLoop(timestamp));
			mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);

			if (mCriticalCheckTimeout.isTimedOut()) {
				mAllSubsystems.forEach(Subsystem::isSystemFaulted);
				mCriticalCheckTimeout.reset();
			}

			if (Constants.LOGGING_ENABLED) {
				generateReport();
				DataReporter.reportOSCData(boundOSCMesage);
			}

			//DEBUG Code
			//Use this code to get accurate loop times for each subsystem individually instead of summed
//			for (int i = 0; i < mAllSubsystems.size(); i++) {
//				Subsystem l = mAllSubsystems.get(i);
//				l.readPeriodicInputs();
//				mLoops.get(i).onLoop(timestamp);
//				l.writePeriodicOutputs();
//				if (mCriticalCheckTimeout.isTimedOut()) {
//					l.isSystemFaulted();
//				}
//			}
//			generateReport();
//			DataReporter.reportOSCData(boundOSCMesage);
//			if (mCriticalCheckTimeout.isTimedOut()) {
//				mCriticalCheckTimeout.reset();
//			}
		}

		@Override
		public void onStop(double timestamp) {
			mLoops.forEach((l) -> l.onStop(timestamp));
		}

		@Override
		public String getName() {
			return "SubsystemManager";
		}
	}

	private class DisabledLoop implements Loop {

		@Override
		public void onFirstStart(double timestamp) {

		}

		@Override
		public void onStart(double timestamp) {

		}

		@Override
		public void onLoop(double timestamp) {
			eLoopTimer.start();
			mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
			mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);

			if (mCriticalCheckTimeout.isTimedOut()) {
				mAllSubsystems.forEach(Subsystem::isSystemFaulted);
				mCriticalCheckTimeout.reset();
			}

			if (Constants.LOGGING_ENABLED) {
				generateReport();
				DataReporter.reportOSCData(boundOSCMesage);
			}

			//DEBUG Code
			//Use this code to get accurate loop times for each subsystem individually instead of summed
//			mAllSubsystems.forEach((l) -> {
//					l.readPeriodicInputs();
//					l.writePeriodicOutputs();
//					if (mCriticalCheckTimeout.isTimedOut()) {
//						l.isSystemFaulted();
//					}
//			});
//			generateReport();
//			DataReporter.reportOSCData(boundOSCMesage);
//			if (mCriticalCheckTimeout.isTimedOut()) {
//				mCriticalCheckTimeout.reset();
//			}
		}

		@Override
		public void onStop(double timestamp) {

		}

		@Override
		public String getName() {
			return "SubsystemManager";
		}
	}

	public void registerEnabledLoops(Looper enabledLooper) {
		mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
		enabledLooper.register(new EnabledLoop());
		mLooperReports.add(enabledLooper);
	}

	public void registerDisabledLoops(Looper disabledLooper) {
		disabledLooper.register(new DisabledLoop());
		mLooperReports.add(disabledLooper);
	}

	@Override
	public void register(Loop loop) {
		mLoops.add(loop);
	}
}
