// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FRC5010.Controller;
import frc.robot.constants.ControlConstants;
import frc.robot.subsystems.DriveTrainMain;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private boolean autoBalanceXMode; 
  private boolean autoBalanceYMode;  

  // These values are received from joystick
  private double xAxisRate;
  private double yAxisRate;
  private double pitchAngleDegrees;
  private double rollAngleDegrees;


  private Controller joystick;   
  private DriveTrainMain driveSubsystem; 
  
  private int offBalanceThreshold = ControlConstants.offBalanceThreshold; 
  private int onBalanceThreshold = ControlConstants.onBalanceThreshold;
  private AHRS ahrs = new AHRS(Port.kMXP); 


  public AutoBalance(Controller joystick, DriveTrainMain driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = joystick; 
    this.driveSubsystem = driveSubsystem;

    addRequirements(this.driveSubsystem);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAxisRate = joystick.getLeftXAxis();
    double yAxisRate = joystick.getLeftYAxis();
    double pitchAngleDegrees = ahrs.getPitch();
    double rollAngleDegrees  = ahrs.getRoll();

     if ( !autoBalanceXMode && 
                 (Math.abs(pitchAngleDegrees) >= 
                  Math.abs(offBalanceThreshold))) {
                autoBalanceXMode = true;
            }
            else if ( autoBalanceXMode && 
                      (Math.abs(pitchAngleDegrees) <= 
                       Math.abs(onBalanceThreshold))) {
                autoBalanceXMode = false;
            }
            if ( !autoBalanceYMode && 
                 (Math.abs(pitchAngleDegrees) >= 
                  Math.abs(offBalanceThreshold))) {
                autoBalanceYMode = true;
            }
            else if ( autoBalanceYMode && 
                      (Math.abs(pitchAngleDegrees) <= 
                       Math.abs(onBalanceThreshold))) {
                autoBalanceYMode = false;
            }
            
            // Control drive system automatically, 
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle
            
            if ( autoBalanceXMode ) {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                xAxisRate = Math.sin(pitchAngleRadians) * -1;
            }
            if ( autoBalanceYMode ) {
                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
                yAxisRate = Math.sin(rollAngleRadians) * -1;
            }

            //driveSubsystem.arcadeDrive(xAxisRate, 0);

          System.out.println("Gyro Angle: " + pitchAngleDegrees); 
          System.out.println("Gyro Angle: " + rollAngleDegrees); 
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
