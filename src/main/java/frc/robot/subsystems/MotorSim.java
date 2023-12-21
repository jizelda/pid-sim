// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSim extends SubsystemBase {

  DCMotorSim _mechanisem; 
  PIDController _controller;
  double[] _delayArray;
  final double delaySecs = 0.5;
  final int delayRoborioPeriods = (int)(delaySecs / 0.02);
  int i = 0;
  static int id;
  /** Creates a new motorSim. */
  public MotorSim(double kP, double kD) {
    _mechanisem = new DCMotorSim(DCMotor.getMiniCIM(2),33,5);

    _controller = new PIDController(kP, 0, kD);

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
    // SmartDashboard.putNumber("pos p:"+ id,_mechanisem.getAngularPositionRotations());
    _controller.setSetpoint(SmartDashboard.getNumber("setpoint", 0));
    // This method will be called once per scheduler run

    // if(SmartDashboard.getBoolean("reset", false)) {
    //   _controller.reset();
    //   _mechanisem.setState(0,0);
    //   SmartDashboard.putBoolean("reset", false);
    // }
  }

  public double getPos() {
    return _mechanisem.getAngularPositionRotations();
  }

}