// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  Timer timer = new Timer();
  double frequency = 0;

  Color color = Color.kWhite;
  LEDPattern pattern = LEDPattern.solid(color);

  int ledStripID, size;
  public LED(int ledStripID, int size) {
    this.ledStripID = ledStripID;
    this.size = size;

    ledStrip = new AddressableLED(this.ledStripID);
    ledBuffer = new AddressableLEDBuffer(this.size);

    ledStrip.setLength(size);
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    timer.start();
  }

  public void setColor(Color color){
    this.color = color;
  }

  public void setPattern(LEDPattern pattern){
    this.pattern = pattern;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pattern.applyTo(ledBuffer);
  
    ledStrip.setData(ledBuffer);
  
  }
}
