package frc.robot.utils;

import java.util.function.BooleanSupplier;

public class RightAnalogXAxisMoved implements BooleanSupplier {
    private final LogitechGamingPad mOpPad;
    private final double mTolerance;
    public RightAnalogXAxisMoved(LogitechGamingPad opPad, double tolerance) {
        this.mOpPad = opPad;
        this.mTolerance = tolerance;
    }

    public RightAnalogXAxisMoved(LogitechGamingPad opPad) {
        this(opPad, 0.05);
    }

    @Override
    public boolean getAsBoolean() {
        return Math.abs(this.mOpPad.getRightAnalogXAxis()) > this.mTolerance;
    }
}