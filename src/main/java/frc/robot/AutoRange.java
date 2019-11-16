package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AutoRange implements PIDOutput, PIDSource {

    private final double pDrive = 0.05;
    private final double iDrive = 0.0;
    private final double dDrive = 0.0;
    private final double fDrive = 0.0;

    private final double tolerancePercent = 0.25;

    private double driveRate;

    PIDController range = new PIDController(pDrive, iDrive, dDrive, fDrive, this, this);

    public AutoRange() {

    }

    public PIDController getRange() {
        return range;
    }

    public double getTolerance() {
        return tolerancePercent;
    }

    public double getDriveRate() {
        return driveRate;
    }

    @Override
    public void pidWrite(double output) {
        if (range.onTarget()) {
            driveRate = 0;
        } else {
            driveRate = output;
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
        return Robot.ta;
    }
}
