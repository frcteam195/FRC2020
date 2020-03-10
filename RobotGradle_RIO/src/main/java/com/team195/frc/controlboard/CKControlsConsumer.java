package com.team195.frc.controlboard;

import com.team195.lib.drivers.dashjoy.CKDashJoystick;

import java.util.function.BiConsumer;

public interface CKControlsConsumer extends BiConsumer<CKDashJoystick, Integer> {

}
