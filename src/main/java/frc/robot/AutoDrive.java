package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AutoDrive implements PIDOutput, PIDSource {

    private final double pDrive = 0.0;
    private final double iDrive = 0.0;
    private final double dDrive = 0.0;
    private final double fDrive = 0.0;

    private final double toleranceTicks = 25.0;

    private double driveRate;

    PIDController drive = new PIDController(pDrive, iDrive, dDrive, fDrive, this, this);

    public AutoDrive() {

    }

    public PIDController getDrive() {
        return drive;
    }

    public double getTolerance() {
        return toleranceTicks;
    }

    public double getDriveRate() {
        return driveRate;
    }

    @Override
    public void pidWrite(double output) {
        if (drive.onTarget()) {
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
        return null;
    }

    @Override
    public double pidGet() {
        return Robot.rightOne.getSelectedSensorPosition();
    }
}