package com.team195.frc.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getTurn();

    boolean getPoopyShoot();

    boolean getQuickTurn();

    boolean getOpenJaw();

    boolean getShoot();
}