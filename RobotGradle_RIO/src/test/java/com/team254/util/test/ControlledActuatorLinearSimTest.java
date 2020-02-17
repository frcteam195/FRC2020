package com.team254.util.test;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class ControlledActuatorLinearSimTest {
    private static final double EPSILON = 1e-8;

    @Test
    public void testItMoves() {
        ControlledActuatorLinearSim sim = new ControlledActuatorLinearSim(0, 1, 1);
        sim.setCommandedPosition(1);
        double now = sim.update(1);
        assertEquals(1, now, EPSILON);
    }

    @Test
    public void testItMovesCorrectly() {
        ControlledActuatorLinearSim sim = new ControlledActuatorLinearSim(0, 1, 1);
        sim.setCommandedPosition(1);
        for (double ts = 0; ts < 1.0; ts += 0.05) {
            double now = sim.update(0.05);
            assertEquals(ts + 0.05, now, EPSILON);
        }

    }

    @Test
    public void testItHitsMinLimit() {
        ControlledActuatorLinearSim sim = new ControlledActuatorLinearSim(0, 1, 1);
        sim.setCommandedPosition(-1);
        for (double ts = 0; ts < 1.0; ts += 0.05) {
            double now = sim.update(0.05);
            assertEquals(0, now, EPSILON);
        }
    }

    @Test
    public void testItHitsMaxLimit() {
        ControlledActuatorLinearSim sim = new ControlledActuatorLinearSim(0, 1, 1);
        sim.setCommandedPosition(2);
        for (double ts = 0; ts < 1.0; ts += 0.05) {
            double now = sim.update(0.05);
            double raw = ts + 0.05;
            double expected = (raw > 1 ? 1 : raw);
            assertEquals(expected, now, EPSILON);
        }
    }
}
