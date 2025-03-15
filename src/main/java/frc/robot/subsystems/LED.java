// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;

  int ledStripID, size;
  public LED(int ledStripID, int size) {
    this.ledStripID = ledStripID;
    this.size = size;

    ledStrip = new AddressableLED(this.ledStripID);
    ledBuffer = new AddressableLEDBuffer(this.size);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
