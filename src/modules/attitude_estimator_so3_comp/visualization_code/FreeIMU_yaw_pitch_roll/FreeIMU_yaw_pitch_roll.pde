/**
Visualize orientation information from a FreeIMU device

INSTRUCTIONS: 
This program has to be run when you have the FreeIMU_serial
program running on your Arduino and the Arduino connected to your PC.
Remember to set the serialPort variable below to point to the name the
Arduino serial port has in your system. You can get the port using the
Arduino IDE from Tools->Serial Port: the selected entry is what you have
to use as serialPort variable.

Copyright (C) 2011-2012 Fabio Varesano - http://www.varesano.net/

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

import processing.serial.*;

Serial myPort;  // Create object from Serial class


float [] q = new float [4];
float [] Euler = new float [3]; // psi, theta, phi

int lf = 10; // 10 is '\n' in ASCII
byte[] inBuffer = new byte[22]; // this is the number of chars on each line from the Arduino (including /r/n)

PFont font;
final int VIEW_SIZE_X = 800, VIEW_SIZE_Y = 600;

int burst = 32, count = 0;

void myDelay(int time) {
  try {
    Thread.sleep(time);
  } catch (InterruptedException e) { }
}

void setup() 
{
  size(VIEW_SIZE_X, VIEW_SIZE_Y, P3D);
  myPort = new Serial(this, "/dev/ttyUSB9", 115200);  
  
  // The font must be located in the sketch's "data" directory to load successfully
  font = loadFont("CourierNew36.vlw"); 
  
  println("Waiting IMU..");
  
  myPort.clear();
  
  while (myPort.available() == 0) {
    myPort.write("v");
    myDelay(1000);
  }
  println(myPort.readStringUntil('\n'));
  myPort.write("q" + char(burst));
  myPort.bufferUntil('\n');
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];
  
  if(inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }
      
  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}


void serialEvent(Serial p) {
  if(p.available() >= 18) {
    String inputString = p.readStringUntil('\n');
    //print(inputString);
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      if(inputStringArr.length >= 5) { // q1,q2,q3,q4,\r\n so we have 5 elements
        q[0] = decodeFloat(inputStringArr[0]);
        q[1] = decodeFloat(inputStringArr[1]);
        q[2] = decodeFloat(inputStringArr[2]);
        q[3] = decodeFloat(inputStringArr[3]);
      }
    }
    count = count + 1;
    if(burst == count) { // ask more data when burst completed
      p.write("q" + char(burst));
      count = 0;
    }
  }
}



void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);
  
  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);
  
  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);
  
  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);
  
  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);
  
  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);
  
  endShape();
}


void drawcompass(float heading, int circlex, int circley, int circlediameter) {
  noStroke();
  ellipse(circlex, circley, circlediameter, circlediameter);
  fill(#ff0000);
  ellipse(circlex, circley, circlediameter/20, circlediameter/20);
  stroke(#ff0000);
  strokeWeight(4);
  line(circlex, circley, circlex - circlediameter/2 * sin(-heading), circley - circlediameter/2 * cos(-heading));
  noStroke();
  fill(#ffffff);
  textAlign(CENTER, BOTTOM);
  text("N", circlex, circley - circlediameter/2 - 10);
  textAlign(CENTER, TOP);
  text("S", circlex, circley + circlediameter/2 + 10);
  textAlign(RIGHT, CENTER);
  text("W", circlex - circlediameter/2 - 10, circley);
  textAlign(LEFT, CENTER);
  text("E", circlex + circlediameter/2 + 10, circley);
}


void drawAngle(float angle, int circlex, int circley, int circlediameter, String title) {
  angle = angle + PI/2;
  
  noStroke();
  ellipse(circlex, circley, circlediameter, circlediameter);
  fill(#ff0000);
  strokeWeight(4);
  stroke(#ff0000);
  line(circlex - circlediameter/2 * sin(angle), circley - circlediameter/2 * cos(angle), circlex + circlediameter/2 * sin(angle), circley + circlediameter/2 * cos(angle));
  noStroke();
  fill(#ffffff);
  textAlign(CENTER, BOTTOM);
  text(title, circlex, circley - circlediameter/2 - 30);
}

void draw() {  

  quaternionToYawPitchRoll(q, Euler);
  
  background(#000000);
  fill(#ffffff);
  
  textFont(font, 20);
  //float temp_decoded = 35.0 + ((float) (temp + 13200)) / 280;
  //text("temp:\n" + temp_decoded + " C", 350, 250);
  textAlign(LEFT, TOP);
  text("Q:\n" + q[0] + "\n" + q[1] + "\n" + q[2] + "\n" + q[3], 20, 10);
  text("Euler Angles:\nYaw (psi)  : " + degrees(Euler[0]) + "\nPitch (theta): " + degrees(Euler[1]) + "\nRoll (phi)  : " + degrees(Euler[2]), 200, 10);
  
  drawcompass(Euler[0], VIEW_SIZE_X/2 - 250, VIEW_SIZE_Y/2, 200);
  drawAngle(Euler[1], VIEW_SIZE_X/2, VIEW_SIZE_Y/2, 200, "Pitch:");
  drawAngle(Euler[2], VIEW_SIZE_X/2 + 250, VIEW_SIZE_Y/2, 200, "Roll:");
}


void quaternionToYawPitchRoll(float [] q, float [] ypr) {
  float gx, gy, gz; // estimated gravity direction
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1);
  ypr[1] = atan2(gx, sqrt(gy*gy + gz*gz));
  ypr[2] = atan2(gy, sqrt(gx*gx + gz*gz));
}


