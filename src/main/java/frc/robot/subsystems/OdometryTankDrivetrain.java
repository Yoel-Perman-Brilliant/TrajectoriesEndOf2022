package frc.robot.subsystems;

import com.spikes2212.command.drivetrains.TankDrivetrain;
import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public abstract class OdometryTankDrivetrain extends TankDrivetrain {

    protected final DifferentialDriveOdometry odometry;
    protected final DifferentialDriveKinematics kinematics;
    protected final RamseteController ramseteController;
    protected final Field2d field2d;

    protected PIDSettings leftPIDSettings;
    protected PIDSettings rightPIDSettings;
    protected FeedForwardSettings ffSettings;

    public OdometryTankDrivetrain(String namespaceName, MotorController left, MotorController right, double width) {
        super(namespaceName, left, right);
        odometry = new DifferentialDriveOdometry(getRotation2d());
        kinematics = new DifferentialDriveKinematics(width);
        ramseteController = new RamseteController();
        field2d = new Field2d();
    }

    @Override
    public void configureDashboard() {
        rootNamespace.putData("field 2d", field2d);
    }

    public abstract Rotation2d getRotation2d();

    public void setVoltage(double leftVoltage, double rightVoltage) {
        leftController.setVoltage(leftVoltage);
        rightController.setVoltage(rightVoltage);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public abstract double getLeftDistance();

    public abstract double getRightDistance();

    public abstract double getLeftSpeed();

    public abstract double getRightSpeed();

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSpeed(), getRightSpeed());
    }

    @Override
    public void periodic() {
        super.periodic();
        odometry.update(getRotation2d(), getLeftDistance(), getRightDistance());
        field2d.setRobotPose(getPose());
    }

    public void resetOdometry(double x, double y, double angle) {
        odometry.resetPosition(new Pose2d(x, y, Rotation2d.fromDegrees(angle)), Rotation2d.fromDegrees(angle));
    }

    public PIDSettings getLeftPIDSettings() {
        return leftPIDSettings;
    }

    public PIDSettings getRightPIDSettings() {
        return rightPIDSettings;
    }

    public FeedForwardSettings getFFSettings() {
        return ffSettings;
    }

    public RamseteController getRamseteController() {
        return ramseteController;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }
}

