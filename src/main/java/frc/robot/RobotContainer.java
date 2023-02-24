// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.LowPosition;
import frc.robot.commands.MidPosition;
import frc.robot.commands.ArmSafetyPosition;
import frc.robot.commands.HighPosition;
import frc.robot.commands.ManualElevatorDrive;
import frc.robot.commands.Test;
import frc.robot.commands.ZeroPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
  ElevatorSubsystem elevSub = new ElevatorSubsystem(); 
  XboxController joystick = new XboxController(OperatorConstants.JOYSTICK);
  double lowEnc = -7;    // TEST
  double midEnc = 120; // TEST
  double highEnc = 190;  // TEST

  LowPosition lowCmd = new LowPosition(elevSub);
  MidPosition midCmd = new MidPosition(elevSub);
  HighPosition highCmd = new HighPosition(elevSub);
  ArmSafetyPosition safe = new ArmSafetyPosition(elevSub);
  ZeroPosition zero = new ZeroPosition(elevSub);
  
  ManualElevatorDrive manualUp = new ManualElevatorDrive(elevSub, 0.2); // 0.2 IS THE MANUAL ELEVATOR SPEED
  ManualElevatorDrive manualDown = new ManualElevatorDrive(elevSub, -0.2);
  
  public RobotContainer() {
    elevSub.setDefaultCommand(new Test(elevSub,() ->  joystick.getLeftY()));

    configureBindings();
  }

  //BUTTONS TO SET THE ELEVATOR TO THEIR POSITIONS
  private void configureBindings() {
    new JoystickButton(joystick, 4).onTrue(highCmd); // high position
    new JoystickButton(joystick, 3).
    onTrue(midCmd); // mid position
    new JoystickButton(joystick, 1).onTrue(lowCmd); // low position
    new JoystickButton(joystick, 7).onTrue(safe);
    new JoystickButton(joystick, 8).onTrue(zero);

    new JoystickButton(joystick, 5).whileTrue(manualUp); // manually moving elevator up
    new JoystickButton(joystick, 6).whileTrue(manualDown); // manually moving elevator down
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
