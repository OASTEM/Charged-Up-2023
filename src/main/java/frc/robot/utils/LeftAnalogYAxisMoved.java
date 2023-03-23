package frc.robot.utils;

import java.util.function.BooleanSupplier;

public class LeftAnalogYAxisMoved implements BooleanSupplier {
    private final LogitechGamingPad mOpPad;
    private final double mTolerance;
    public LeftAnalogYAxisMoved(LogitechGamingPad opPad, double tolerance) {
        this.mOpPad = opPad;
        this.mTolerance = tolerance;
    }

    public LeftAnalogYAxisMoved(LogitechGamingPad opPad) {
        this(opPad, 0.05);
    }

    @Override
    public boolean getAsBoolean() {
        return Math.abs(this.mOpPad.getLeftAnalogYAxis()) > this.mTolerance;
    }
}
