package com.team195.frc.auto.modes;

import com.team195.frc.auto.AutoModeEndedException;
import com.team195.frc.auto.AutoModeBase;
import com.team195.frc.auto.actions.OpenLoopDrive;
import com.team195.frc.auto.actions.WaitAction;
import com.team195.frc.reporters.ConsoleReporter;

public class CrossAutoLineMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        ConsoleReporter.report("Running Cross auto line");
        runAction(new WaitAction(5.0));
        runAction(new OpenLoopDrive(-0.3, -0.3, 5.0));
    }
}
