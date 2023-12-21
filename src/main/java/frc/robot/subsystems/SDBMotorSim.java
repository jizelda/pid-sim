// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SDBMotorSim extends SubsystemBase {

  DCMotorSim _mechanisem; 
  PIDController _controller;
  double[] _delayArray;
  final double delaySecs = 0.5;
  final int delayRoborioPeriods = (int)(delaySecs / 0.02);
  final String _name;
  int i = 0;
  /** Creates a new motorSim. */
  public SDBMotorSim(String name) {
    _name = name;
    _mechanisem = new DCMotorSim(DCMotor.getMiniCIM(2),33,5);
    _controller = new PIDController(5, 0, 0);

    SmartDashboard.putNumber("kP"+_name, 0);
    SmartDashboard.putNumber("kI"+_name, 0);
    SmartDashboard.putNumber("kD"+_name, 0);

    _delayArray = new double[delayRoborioPeriods];

    for(int j = 0; j < _delayArray.length;j++) {
      _delayArray[j] = 0;
    }
  }

  @Override
  public void periodic() {
    _mechanisem.setInput(_controller.calculate(_delayArray[i % _delayArray.length]));
    _delayArray[i%_delayArray.length] = _mechanisem.getAngularPositionRotations();
    i++;
    _mechanisem.update(0.02);
    SmartDashboard.putNumber("pos"+_name,_mechanisem.getAngularPositionRotations());
    _controller.setP(SmartDashboard.getNumber("kP"+_name, 0));
    _controller.setI(SmartDashboard.getNumber("kI"+_name, 0));
    _controller.setD(SmartDashboard.getNumber("kD"+_name, 0));
    _controller.setSetpoint(SmartDashboard.getNumber("setpoint", 0));
    // This method will be called once per scheduler run
  }

}