import processing.serial.*;
import processing.opengl.*;

Arrow xArrow, yArrow, zArrow;

/**
 ** Display settings
 **/
 
int sizeX = 700,sizeY = 700;
// System

/**
 ** Box settings
 **/

int  fixedFrameL = 300;

/**
 ** Serial settings
 **/
Serial port; // The serial port

/**
 ** Orientation variables
 **/
float[] q = new float[4];
//Quaternion quat = new Quaternion(1, 0, 0, 0);
float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];
int interval = 0;
int rotzfactor = 0;
int rotxfactor = 0;
int rotyfactor = 0;


void setup() 
{
  // 300px square viewport using OpenGL rendering
  size(sizeX, sizeY, OPENGL);  
  lights();
  smooth();
  
  // Reference system
  xArrow = new Arrow(80,55,10,175,65,45);
  yArrow = new Arrow(sizeX/2,+sizeX/2+100,sizeY/2+10,sizeY/2+175,150,200);
  zArrow = new Arrow(0,100,0,12,0,200);
  
  // Serial 
  
  println(Serial.list());
  // get the first available port (use EITHER this OR the specific port code below)
  String portName = Serial.list()[0];
  // get a specific serial port (use EITHER this OR the first-available code above)
  //String portName = "COM4";
  // open the serial port
  port = new Serial(this, portName, 9600);
  
  //port.write('r');
  /*
  if ( port.available() > 0) 
  {  // If data is available,
    String val = port.readStringUntil('\n');   
      // read it and store it in val
      if (1  == Integer.parseInt(val))
      println("Connected");
  } 
  println(val); //print it out in the console
  */
}

void draw() 
{
  setCamera();
  {
    // resend single character to trigger DMP init/start
    // in case the MPU is halted/reset while applet is running
    port.write('r');
    interval = millis();
  }
  
  //float rotx = (mouseY/360.0)*-2*PI+PI;
  //float roty = (mouseX/420.0)*2*PI-PI;
  float rotx = rotxfactor*PI/36;
  float roty = rotyfactor*PI/36;
  float rotz = rotzfactor*PI/36;

  background(0);
  stroke(0, 200, 200);
  line (0,20,sizeX,20); // monitor bar
  fill(255);
  textSize(10);
  text ("Rotation:", sizeX*0.05,sizeY*0.1-20);
  text (" X: " + rotx*180/PI + "°"  , sizeX*0.05,sizeY*0.1);
  text (" Y: " + roty*180/PI  + "°", sizeX*0.05,sizeY*0.1+20);
  text (" Z: "+ rotzfactor+" pi/36) ", sizeX*0.05,sizeY*0.1+40);
  
  pushMatrix();
    translate(sizeX*0.8,sizeY*0.2);
    text("x",50,10);
    stroke(255,0,0);
    line(0,0,0,50,0,0);
    text("y",10,50);
    stroke(0,255,0);
    line(0,0,0,0,50,0);
    text("z",20,20);
    stroke(255,0,255);
    line(0,0,0,0,0,100);
  popMatrix();
  
  fill(0, 0, 200);
  
  textSize(12);
  text ("Quadcopter Orientation by UserK ", sizeX*0.7,sizeY*0.95);
  
    translate(sizeX/2, sizeY/2, 0); // center drawing start point in screen
   
  strokeWeight(1);
  stroke(50);
  line(-fixedFrameL/2, 0, fixedFrameL/2, 0);       //
  line(0, fixedFrameL/2, 0, -fixedFrameL/2);       // draw stationary axis lines
  line(0, 0, -fixedFrameL/2, 0, 0, fixedFrameL/2); //
  strokeWeight(2);
  stroke(255);
  noFill();
  box(fixedFrameL); // Draw stationary box
  
  
  rotateX(rotx);  //
  rotateY(roty);  // 
  rotateZ(rotz);  //
  textSize(15);
  text("Xs",50,10);
  strokeWeight(2);
  stroke(255,0,0);
  line(0, 0, 50, 0);  
  textSize(15);
    text("Ys",10,50);  
  stroke(0,255,0);   //
  line(0, 50, 0, 0);       // draw the rotating axis lines
  textSize(15);
    text("Zs",20,20);
  stroke(255,0,255);
  line(0, 0, 0, 0, 0, 50); 
  
  stroke(255, 0, 0);
  noFill();
  
  strokeWeight(2);
  stroke(255);
  box(150,10,50);  
}

void setCamera()
{
  
}

void keyPressed() 
{
  //if (key == CODED) 
  //{
    switch (key) 
    {
  case 'r':
  println("pitch+");
    rotxfactor++;
    break;
  case 'e':
  println("pitch-");
    rotxfactor--;
    break; 
  case 'p':
  println("roll+");
    rotyfactor++;
    break;  
  case 'o':
  println("roll-");
    rotyfactor--;
    break;  
  case 'y':
  println("yaw+");
    rotzfactor++;
    break;  
  case 't':
  println("yaw-");
    rotzfactor--;
    break;     
  default:  
    break;
    }
  //}
}

void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -10, -10);
  line(0, 0, 10, -10);
  popMatrix();
} 

public class Arrow {
  float cc_x1, cc_y1, cc_z1, cc_x2, cc_y2, cc_z2;
 
  public Arrow (float x1, float y1,float z1, float x2, float y2,float z2) {
    cc_x1 = x1;
    cc_y1 = y1;
    cc_z1 = z1;
    cc_x2 = x2;
    cc_y2 = y2;
    cc_z2 = z2;
  }
 
  public void drawarrow(int r, int g,int b) {
   stroke(0,0,0);
   stroke(r,0,0);
   line(cc_x1,cc_y1,cc_z1,cc_x2,cc_y2,cc_z2);
  
  pushMatrix();
 /*
 translate(cc_x2, cc_y2);
    float a = atan2(cc_x1-cc_x2, cc_y2-cc_y1);
    stroke(173,139,255);
    rotate(a);  
   stroke(0,g,0);
    line(0, 0,cc_z1, -0.25, -0.25,cc_z2);
   stroke(0,0,b);
    line(0, 0,cc_z1, 0.25, -0.25,cc_z2);
 */
    //translate(cc_x2, cc_y2);
    float a = atan2(cc_x1-cc_x2, cc_y2-cc_y1);
    stroke(r,0,0);
    rotate(a);  
   stroke(0,g,0);
    line(0, 0,cc_z1, -0.25, -0.25,cc_z2);
   stroke(0,0,b);
    line(0, 0,cc_z1, 0.25, -0.25,cc_z2);
    popMatrix();
  }
}
