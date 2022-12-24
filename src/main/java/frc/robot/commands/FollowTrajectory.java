package frc.robot.commands;

import com.spikes2212.control.FeedForwardSettings;
import com.spikes2212.control.PIDSettings;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.OdometryTankDrivetrain;

public class FollowTrajectory extends RamseteCommand {

    public FollowTrajectory(Trajectory trajectory, OdometryTankDrivetrain drivetrain, PIDSettings pidSettings,
                            FeedForwardSettings feedForwardSettings) {
        super(trajectory, drivetrain::getPose, drivetrain.getRamseteController(), getFFController(feedForwardSettings),
                drivetrain.getKinematics(), drivetrain::getWheelSpeeds, getPIDController(pidSettings),
                getPIDController(pidSettings), drivetrain::setVoltage, drivetrain);
    }

    private static PIDController getPIDController(PIDSettings settings) {
        PIDController controller = new PIDController(settings.getkP(), settings.getkI(), settings.getkD());
        controller.setTolerance(settings.getTolerance());
        return controller;
    }

    private static SimpleMotorFeedforward getFFController(FeedForwardSettings settings) {
        return new SimpleMotorFeedforward(settings.getkS(), settings.getkV(), settings.getkA());
    }
}
