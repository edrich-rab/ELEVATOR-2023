// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
  boolean pidOn = true;
  CANSparkMax elevator;
  RelativeEncoder enc;
  DigitalInput topLim;
  DigitalInput bottomLim;
  PIDController pid = new PIDController(0.1, 0, 0);
  double setpoint = enc.getPosition();
  double lastError = 0;
  double manualSpeed = 0;

  public ElevatorSubsystem() {
    topLim = new DigitalInput(OperatorConstants.TOP_LIMIT_SWITCH);
    bottomLim = new DigitalInput(OperatorConstants.BOTTOM_LIMIT_SWITCH);
    elevator = new CANSparkMax(OperatorConstants.ELEVATOR_ID, MotorType.kBrushless);
    enc = elevator.getEncoder();
  }

  public void setManualSpeed(double inputSpeed){
    manualSpeed = inputSpeed;
  }

  public void changeSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void init(){ 
    elevator.setIdleMode(IdleMode.kBrake);
  }

  public boolean topPressed(){
    return !topLim.get();
  }

  public boolean bottomPressed(){
    return !bottomLim.get();
  }

  public void enablePid(){
    pidOn = true;
  }

  public void disablePid(){
    pidOn = false;
  }

  public void currentEncValtoSetpoint(){
    setpoint = enc.getPosition();
  }

  @Override
  public void periodic() {
    double encoderValue = enc.getPosition();

    //
    double currentError = setpoint - encoderValue;

    if(currentError > 0 && lastError < 0){ // FOR I VALUE
      pid.reset();
    }
    else if(currentError < 0 && lastError > 0){
      pid.reset();
    }
    //
    
    double calcSpeed = 0;
    if(pidOn){
      calcSpeed = pid.calculate(encoderValue, setpoint); // SETS MOTOR SPEED TO CALCULATED PID SPEED 
    }
    else{
      calcSpeed = manualSpeed;
    }
    
    //
    if(topPressed() && calcSpeed > 0){ // BUT IF THE TOP SWITCH IS PRESSED, IT RESETS ENCODER AND CHANGES SET POINT TO 0
      calcSpeed = 0;
      //setpoint = enc.getPosition();
    }
    else if(bottomPressed() && calcSpeed < 0){ // -9 < - low position
      calcSpeed = 0;
      //setpoint = enc.getPosition();
    }
    //
    
    if(calcSpeed > 1){ // IF SPEED CALCULATED IS GREATER THAN 1, SETS MAX SPEED TO 1
      calcSpeed = 1;
    }
    else if(calcSpeed < -0.2){ // IF SPEED CALCULATED IS LESS THAN -1, SETS MAX SPEED TO -1
      calcSpeed = -0.2;
    }
    elevator.set(calcSpeed);
    
    

    SmartDashboard.getNumber(" PID Speed", calcSpeed);
    SmartDashboard.getBoolean("Top switch pressed" , topPressed()); 
    SmartDashboard.getBoolean("Bottom switch pressed", bottomPressed());
    SmartDashboard.getNumber("encoder counts", encoderValue);
  }
}