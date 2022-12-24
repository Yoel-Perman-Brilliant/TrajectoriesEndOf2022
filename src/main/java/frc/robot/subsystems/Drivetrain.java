package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

//ghost implementation
public class Drivetrain extends OdometryTankDrivetrain {

    public Drivetrain(String namespaceName, MotorController left, MotorController right, double width) {
        super(namespaceName, left, right, width);
    }

    @Override
    public Rotation2d getRotation2d() {
        return null;
    }

    @Override
    public double getLeftDistance() {
        return 0;
    }

    @Override
    public double getRightDistance() {
        return 0;
    }

    @Override
    public double getLeftSpeed() {
        return 0;
    }

    @Override
    public double getRightSpeed() {
        return 0;
    }
}
