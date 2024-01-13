package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
  private final DriveTrain m_drivetrainSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  // public DefaultDriveCommand(DriveTrain driveTrain, DoubleSupplier
  // translationXSupplier,
  // DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier){
  // this.m_drivetrainSubsystem = driveTrain;
  // this.m_translationXSupplier = translationXSupplier;
  // this.m_translationYSupplier = translationYSupplier;
  // this.m_rotationSupplier = rotationSupplier;

  // addRequirements(driveTrain);
  // }

  public DefaultDriveCommand(DriveTrain driveTrain, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
    this.m_drivetrainSubsystem = driveTrain;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    addRequirements(driveTrain);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
    // field-oriented movement
    m_drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_translationXSupplier.getAsDouble(),
            m_translationYSupplier.getAsDouble(),
            m_rotationSupplier.getAsDouble(),
            m_drivetrainSubsystem.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
