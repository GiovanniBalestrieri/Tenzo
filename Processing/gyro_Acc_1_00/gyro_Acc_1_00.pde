import processing.serial.*;
import processing.opengl.*;


/**
 ** Display settings
 **/
 
int sizeX = 1100,sizeY = 600;
// System

/**
 ** Visual settings
 **/

int  fixedFrameL = 170;


int  quadL = 100;

/**
 ** Serial settings
 **/
Serial port; 
String val;

/**
 ** Orientation variables
 **/
float[] q = new float[4];
//Quaternion quat = new Quaternion(1, 0, 0, 0);
float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];
float[] rpy = new float[3];
int interval = 0;
int rotzfactor = 0;
int rotxfactor = 0;
int rotyfactor = 0;

float rotx,roty,rotz;

void setup() 
{
  // 300px square viewport using OpenGL rendering
  size(sizeX, sizeY, OPENGL);  
  lights();
  smooth();
  // Serial 
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  port = new Serial(this, portName, 115200); 
}

void draw() 
{
  SerialRoutine();
  background(0);
  displayInfos();
  // Camera control
  //translate(0, 0, 0); 
  //rotateY((mouseX/360.0)*-2*PI+PI);
  rotateX(-25*PI/180);
  translate(0,0,200);
  
  drawObjects(); 
  
  interval = millis(); 
}

void SerialRoutine()
{
 if (port.available() > 0) 
  {  
    val = port.readStringUntil('\n'); 
    //println("Received:" + val);
    if (val!=null)
    {
      String[] vals = split(val,",");
      println("composed by: "+vals.length+ " values");
      println("Receiving: "+val);
      if (vals.length==3)
      {
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        rpy[2] =  Float.parseFloat(vals[2]);
        rotx = rpy[0]*PI/180;
        roty = rpy[2]*PI/180;
        rotz = rpy[1]*PI/180;
      }
      if (vals.length==2)
      {
      println("Saving angles ");
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        rotz = rpy[0]*PI/180;
        rotx = rpy[1]*PI/180;
      }
    }
  } 

  if (mousePressed == true) 
  {                           //if we clicked in the window
   //port.write('A');         //send a 1
   //println("A");   
  }
  /*else 
  {                           //otherwise
    port.write('B');          //send a 0
  } */   
}

void displayInfos()
{
  //float rotx = (mouseY/360.0)*-2*PI+PI;
  //float roty = (mouseX/420.0)*2*PI-PI;
  //rotx = rotxfactor*PI/36;
  //roty = rotyfactor*PI/36;
  //rotz = rotzfactor*PI/36;

  stroke(0, 200, 200);
  line (0,20,sizeX,20); // monitor bar
  fill(255);
  textSize(10);
  text ("Rotation:", sizeX*0.05,sizeY*0.1-20);
  text (" X: " + rotx*180/PI + "°"  , sizeX*0.05,sizeY*0.1);
  text (" Y: " + roty*180/PI  + "°", sizeX*0.05,sizeY*0.1+20);
  //text (" Z: "+ rotzfactor*180/36+" pi/36) ", sizeX*0.05,sizeY*0.1+40);
  text (" Z: "+ rotz*180/PI+" pi/36) ", sizeX*0.05,sizeY*0.1+40);
  
  fill(0, 0, 200);
  
  textSize(12);
  text ("Quadcopter Orientation by UserK ", sizeX*0.7,sizeY*0.95);
}

void drawObjects()
{
      pushMatrix();
      
      //Draw axes inertial frame
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
      
      
      pushMatrix();
      
        // Acc
        translate(0,0, 0); 
        translate(sizeX/2*0.5, sizeY/3, 0); 
         
         // Draw body frame
        strokeWeight(1);
        stroke(50);
        line(-fixedFrameL/2, 0, fixedFrameL/2, 0);       //
        line(0, fixedFrameL/2, 0, -fixedFrameL/2);       // draw stationary axis lines
        line(0, 0, -fixedFrameL/2, 0, 0, fixedFrameL/2); //
        strokeWeight(2);
        stroke(0,255,0);
        noFill();
        box(fixedFrameL); // Draw stationary box
        
        
        rotateX(rotx); 
        rotateY(roty); 
        rotateZ(rotz); 
        stroke(255,0,255);
        line(0, 0, 0, 0, 0, 50); 
        
        stroke(255, 0, 0);
        noFill();
        
        strokeWeight(2);
        stroke(255);
        fill(200);
        box(quadL,5,5);
        rotateY(PI/2);
        box(quadL,5,5);
      
      popMatrix();
      
      pushMatrix();
        translate(sizeX/2, sizeY/3, 0); 
         
         // Draw body frame
        strokeWeight(1);
        stroke(50);
        line(-fixedFrameL/2, 0, fixedFrameL/2, 0);       //
        line(0, fixedFrameL/2, 0, -fixedFrameL/2);       // draw stationary axis lines
        line(0, 0, -fixedFrameL/2, 0, 0, fixedFrameL/2); //
        strokeWeight(2);
        stroke(255);
        noFill();
        box(fixedFrameL); // Draw stationary box
        
        
        rotateX(rotx); 
        rotateY(roty); 
        rotateZ(rotz); 
        stroke(255,0,255);
        line(0, 0, 0, 0, 0, 50); 
        
        stroke(255, 0, 0);
        noFill();
        
        strokeWeight(2);
        stroke(255);
        fill(200);
        box(quadL,5,5);
        rotateY(PI/2);
        box(quadL,5,5);
        
      popMatrix();
      
      pushMatrix();
      
        // Acc
        translate(0,0, 0); 
        translate(sizeX*0.73, sizeY/3, 0); 
         
         // Draw body frame
        strokeWeight(1);
        stroke(50);
        line(-fixedFrameL/2, 0, fixedFrameL/2, 0);       //
        line(0, fixedFrameL/2, 0, -fixedFrameL/2);       // draw stationary axis lines
        line(0, 0, -fixedFrameL/2, 0, 0, fixedFrameL/2); //
        strokeWeight(2);
        stroke(255,0,0);
        noFill();
        box(fixedFrameL); // Draw stationary box
        
        
        rotateX(rotx); 
        rotateY(roty); 
        rotateZ(rotz); 
        stroke(255,0,255);
        line(0, 0, 0, 0, 0, 50); 
        
        stroke(255, 0, 0);
        noFill();
        
        strokeWeight(2);
        stroke(255);
        fill(200);
        box(quadL,5,5);
        rotateY(PI/2);
        box(quadL,5,5);      
      
      popMatrix();
      
    popMatrix();
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


void cylinder(int sides,float r, float h)
{
 float angle = 360/sides;
 float halfHeight = h/2;
 
 // draw top shaperrrr
 beginShape();
 for (int i=0; i< sides; i++)
 {
   float x = cos(radians(i*angle))*r;
   float y = sin(radians(i*angle))*r;
   vertex(x,y,-halfHeight);
 } 
 endShape(CLOSE);
 
 // draw bottom shape
 beginShape();
 for (int i=0; i< sides; i++)
 {
   float x = cos(radians(i*angle))*r;
   float y = sin(radians(i*angle))*r;
   vertex(x,y,-halfHeight);
 } 
 endShape(CLOSE);
}
