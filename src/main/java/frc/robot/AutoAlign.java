package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AutoAlign implements PIDOutput, PIDSource {

    private double pTurn = 0.012;
    private double iTurn = 0.0;
    private double dTurn = 0.0078;
    private double fTurn = 0.0;

    private final double toleranceDegrees = 1.0;

    private double turnRate;

    PIDController align = new PIDController(pTurn, iTurn, dTurn, fTurn, this, this);

    public AutoAlign() {

    }

    public PIDController getAlign() {
        return align;
    }

    public double getTolerance() {
        return toleranceDegrees;
    }

    public double getTurnRate() {
        return turnRate;
    }

    @Override
    public void pidWrite(double output) {
        if (align.onTarget()) {
            turnRate = 0;
        } else {
            turnRate = output;
        }
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        return;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return Robot.tx;
    }
}