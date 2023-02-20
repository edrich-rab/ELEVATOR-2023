// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class LowPosition extends CommandBase {
  /** Creates a new ElevatorCommand. */
  ElevatorSubsystem elevSub;
  double setPoint;
  public LowPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = -7;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    elevSub.changeSetpoint(setPoint);
  }

  @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished() {
    if(elevSub.isAtSetpoint()){ // if setpoint is within tolerance return true
      return true;
    }
    else{ // else if not within tolerance return false
      return false;
    }
  }
}
