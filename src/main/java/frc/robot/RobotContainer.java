// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.LowPosition;
import frc.robot.commands.MidPosition;
import frc.robot.commands.HighPosition;
import frc.robot.commands.ManualElevatorDrive;
import frc.robot.commands.Test;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  ElevatorSubsystem elevSub = new ElevatorSubsystem(); 
  XboxController joystick = new XboxController(OperatorConstants.JOYSTICK);
  double lowEnc = -7;    // TEST
  double midEnc = 120; // TEST
  double highEnc = 190;  // TEST


  //ElevatorCommand lowCmd = new ElevatorCommand(elevSub, lowEnc);
  //ElevatorCommand highCmd = new ElevatorCommand(elevSub, highEnc);
  //ElevatorCommand midCmd = new ElevatorCommand(elevSub, midEnc);
  LowPosition lowCmd = new LowPosition(elevSub);
  MidPosition midCmd = new MidPosition(elevSub);

  HighPosition highCmd = new HighPosition(elevSub);
  
  ManualElevatorDrive manualUp = new ManualElevatorDrive(elevSub, 0.2);
  ManualElevatorDrive manualDown = new ManualElevatorDrive(elevSub, -0.2);
  
  public RobotContainer() {
    elevSub.setDefaultCommand(new Test(elevSub,() ->  joystick.getLeftY()));

    configureBindings();
  }

  
  private void configureBindings() {
    new JoystickButton(joystick, 2).onTrue(highCmd);
    new JoystickButton(joystick, 1).onTrue(midCmd);
    new JoystickButton(joystick, 3).onTrue(lowCmd);

    new JoystickButton(joystick, 5).whileTrue(manualUp);
    new JoystickButton(joystick, 6).whileTrue(manualDown);
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
