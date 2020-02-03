package com.team195.frc.auto.modes;

import com.team195.frc.auto.AutoModeEndedException;
import com.team195.frc.auto.AutoModeBase;
import com.team195.frc.reporters.ConsoleReporter;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        ConsoleReporter.report("Doing nothing");
    }
}
