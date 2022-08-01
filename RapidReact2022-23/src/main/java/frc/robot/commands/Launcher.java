// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControlConstants;
import frc.robot.constants.ShooterConstants.HoodConstants;
import frc.robot.subsystems.DriveTrainMain;
import frc.robot.subsystems.ShooterSubsystem;

public class Launcher extends CommandBase {
  private ShooterSubsystem shooterSubsystem;
  private Joystick operator;
  public Launcher(ShooterSubsystem shooterSubsystem, Joystick operator) {
    this.shooterSubsystem = shooterSubsystem;
    this.operator = operator;
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pow = DriveTrainMain.scaleInputs(-operator.getRawAxis(ControlConstants.spinHood))* HoodConstants.manualPow;
    SmartDashboard.putNumber("HoodPow", pow);
    shooterSubsystem.spinHood(pow);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
