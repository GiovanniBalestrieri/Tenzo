import processing.serial.*;
import processing.opengl.*;


/**
 ** Display settings
 **/ 
int sizeX = 1100,sizeY = 600;

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
float[] ypr = new float[8];
float[] rpy = new float[8];
int interval = 0;
int rotzfactor = 0;
int rotxfactor = 0;
int rotyfactor = 0;

float rotx,roty,rotz;
float accRotx,accRoty,accRotz;
float mixRotx,mixRoty,mixRotz;

/**
 ** Orientation variables
 **/
float throttle = 0;
 
 
void setup() 
{
  // 300px square viewport using OpenGL rendering
  size(sizeX, sizeY, OPENGL);  
  lights();
  smooth();
  // Serial 
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  port = new Serial(this, portName, 115200); 
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

}

void draw() 
{
  SerialRoutine();
  background(0);  
  // Camera control

  displayInfos();
  
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
        rotz = -rpy[1]*PI/180;
      }
      if (vals.length==2)
      {
      println("Saving angles ");
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        accRotz = -rpy[0]*PI/180;
        accRotx = -rpy[1]*PI/180;
      }      
      if (vals.length==5)
      {
        println("Saving angles ");
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        rpy[2] =  Float.parseFloat(vals[2]);
        rpy[3] =  Float.parseFloat(vals[3]);
        rpy[4] =  Float.parseFloat(vals[4]);
       
        rotx = rpy[0]*PI/180;
        roty = rpy[2]*PI/180;
        rotz = -rpy[1]*PI/180;
        accRotz = -rpy[3]*PI/180;
        accRotx = -rpy[4]*PI/180;
        /*
         rotx = rpy[0]*PI/180;
        roty = rpy[2]*PI/180;
        rotz = rpy[1]*PI/180;
        accRotz = rpy[3]*PI/180;
        accRotx = -rpy[4]*PI/180;
        */
      }
      if (vals.length==8)
      {
        println("Acc - Gyro - Fusion ");
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        rpy[2] =  Float.parseFloat(vals[2]);
        rpy[3] =  Float.parseFloat(vals[3]);
        rpy[4] =  Float.parseFloat(vals[4]);
        rpy[5] =  Float.parseFloat(vals[5]);
        rpy[6] =  Float.parseFloat(vals[6]);
        rpy[7] =  Float.parseFloat(vals[7]);
        // Acc
        accRotz = -rpy[3]*PI/180;
        accRotx = -rpy[4]*PI/180;
        // Gyro
        rotx = -rpy[0]*PI/180;
        roty = rpy[2]*PI/180;
        rotz = -rpy[1]*PI/180;
        // Complementary
        mixRotx = -rpy[6]*PI/180;
        mixRoty = rpy[7]*PI/180;
        mixRotz = -rpy[5]*PI/180;
      }
      if (vals.length==9)
      {
        println("Acc - Gyro - Fusion ");
        rpy[0] =  Float.parseFloat(vals[0]);
        rpy[1] =  Float.parseFloat(vals[1]);
        rpy[2] =  Float.parseFloat(vals[2]);
        rpy[3] =  Float.parseFloat(vals[3]);
        rpy[4] =  Float.parseFloat(vals[4]);
        rpy[5] =  Float.parseFloat(vals[5]);
        rpy[6] =  Float.parseFloat(vals[6]);
        //rpy[7] =  Float.parseFloat(vals[7]);
        throttle =  Float.parseFloat(vals[8]);
        rotx = rpy[0]*PI/180;
        roty = rpy[2]*PI/180;
        rotz = -rpy[1]*PI/180;
        accRotz = -rpy[3]*PI/180;
        accRotx = -rpy[4]*PI/180;
        mixRotx = -rpy[6]*PI/180;
        //mixRoty = rpy[7]*PI/180;
        mixRotz = -rpy[5]*PI/180;
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
  stroke(0, 200, 200);
  line (0,20,sizeX,20); // monitor bar
  fill(255);
  textSize(10);
  text ("Rotation:", sizeX*0.25,sizeY*0.1-20);
  text (" X: " + accRotx*180/PI + "°"  , sizeX*0.25,sizeY*0.1);
  text (" Y: " + accRoty*180/PI  + "°", sizeX*0.25,sizeY*0.1+20);
  //text (" Z: "+ rotzfactor*180/36+" pi/36) ", sizeX*0.25,sizeY*0.1+40);
  text (" Z: "+ accRotz*180/PI+" pi/36) ", sizeX*0.25,sizeY*0.1+40);
  
  text ("Rotation:", sizeX*0.50,sizeY*0.1-20);
  text (" X: " + rotx*180/PI + "°"  , sizeX*0.50,sizeY*0.1);
  text (" Y: " + roty*180/PI  + "°", sizeX*0.50,sizeY*0.1+20);
  //text (" Z: "+ rotzfactor*180/36+" pi/36) ", sizeX*0.50,sizeY*0.1+40);
  text (" Z: "+ rotz*180/PI+" pi/36) ", sizeX*0.50,sizeY*0.1+40);
  
  text ("Rotation:", sizeX*0.75,sizeY*0.1-20);
  text (" X: " + mixRotx*180/PI + "°"  , sizeX*0.75,sizeY*0.1);
  text (" Y: " + mixRoty*180/PI  + "°", sizeX*0.75,sizeY*0.1+20);
  //text (" Z: "+ rotzfactor*180/36+" pi/36) ", sizeX*0.75,sizeY*0.1+40);
  text (" Z: "+ rotz*180/PI+" pi/36) ", sizeX*0.75,sizeY*0.1+40);
  
  fill(0, 0, 200);
  textSize(29);
  text (throttle, sizeX*0.15,sizeY*0.90);
  
  textSize(12);
  text ("Quadcopter Orientation by UserK ", sizeX*0.7,sizeY*0.95);
}

void drawObjects()
{
  
  rotateX(-25*PI/180);
    pushMatrix();
    camera(mouseX, mouseY, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);
  translate(0,0,200);
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
      
      
      rotateX(accRotx); 
      rotateY(accRoty); 
      rotateZ(accRotz); 
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
      
      
      rotateX(mixRotx); 
      rotateY(mixRoty); 
      rotateZ(mixRotz); 
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
      case '1':
    println("test M1");
    port.write("1");
    break;
    case '2':
  println("testM2");
    port.write("2");
    break;
    case '3':
  println("testM3");
    port.write("3");
    break;
    case '4':
  println("testM4");
    port.write("4");
    break;
      case 'k':
  println("takeoff+");
    port.write("a");
    break;
  case 'L':
  println("Landing");
    port.write("L");
    break; 
  case 'w':
  println("+5");
    port.write("w");
    break; 
  case 's':
    println("-5");
    port.write("s");
    break; 
  case 'e':
  println("+1");
    port.write("e");
    break; 
  case 'd':
    println("-1");
    port.write("d");
    break; 
  case 'p':
    println("Enable Pid");
    port.write("p");
    break; 
  case 'l':
    println("DISABLE Pid");
    port.write("l");
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
