// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  AddressableLED ledStrip;
  AddressableLEDBuffer ledBuffer;
  Timer timer = new Timer();
  double frequency = 0;

  int r = 0, g = 0, b = 0;
  //int i = 0;
  String pattern;

  int ledStripID, size;
  public LED(int ledStripID, int size) {
    this.ledStripID = ledStripID;
    this.size = size;

    ledStrip = new AddressableLED(this.ledStripID);
    ledBuffer = new AddressableLEDBuffer(this.size);

    for(int i = 0; i < this.size; i++){
      ledBuffer.setRGB(i, 255, 255, 255);
    }

    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  public void setSolidColor(int r, int g, int b){
    pattern = "solid";
    this.r = r;
    this.g = g;
    this.b = b;
  }

  public void setBlinkColor(int r, int g, int b, double frequency){
    pattern = "blink";
    this.r = r;
    this.g = g;
    this.b = b;
    this.frequency = frequency;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (pattern) {
      case "solid":
        for(int i = 0; i < this.size; i++){
          ledBuffer.setRGB(i, r, g, b);
        }
        break;
    
      case "blink":
        if(timer.get() < frequency / 2){
          for(int i = 0; i < this.size; i++){
            ledBuffer.setRGB(i, r, g, b);
          }
        }
        else if(timer.get() < frequency){
          for(int i = 0; i < this.size; i++){
            ledBuffer.setRGB(i, 0, 0, 0);
          }
        }
        else{
          timer.reset();
        }
        break;

      default:
        for(int i = 0; i < this.size; i++){
          ledBuffer.setRGB(i, 255, 255, 255);
        }
        break;
    }
  
    ledStrip.setData(ledBuffer);
  
  }
}
