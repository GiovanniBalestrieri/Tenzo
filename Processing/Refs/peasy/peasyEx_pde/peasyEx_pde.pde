import peasy.*;

PeasyCam cam;
int fixedFrameL = 100;
PVector camPos;

/**
 ** Window
 **/
int sizeX = 700, sizeY = 700;

PVector[] points = new PVector[50];

void setup() {
  size(sizeX,sizeY,P3D);
  cam = new PeasyCam(this, 100);
  float[] position = cam.getPosition();
  cam.setMinimumDistance(50);
  cam.setMaximumDistance(500);
  cam.setYawRotationMode();  // like spinning a globe
//cam.setPitchRotationMode(); 
//cam.lookAt(20,50,60);// double y, double z);
}
void draw() 
{
  background(0);
  displayInfos();  
  rotateX(-25*PI/180);
  translate(0,0,100);  
  drawObjects();
  // Draw body frame
  strokeWeight(1);
  stroke(50);
  
  // axes for frame of reference
  stroke(255,0,0);         // red = Z
  line(0,0,-fixedFrameL,0,0,fixedFrameL);
  stroke(0,255,0);         // green = Y 
  line(0,-fixedFrameL,0,0,fixedFrameL,0);
  stroke(0,0,255);         // blue = X
  line(-fixedFrameL,0,0,fixedFrameL,0,0);
}
void drawPoints()
{
 // points on axes to denote positive orientation
  strokeWeight(9);
  point(40,0,0);
  point(0,40,0);
  point(0,0,40);
  strokeWeight(1);
}

void drawObjects()
{
  fill(255,0,0);
  box(30);
  pushMatrix();
  drawPoints();
  translate(0,0,20);
  fill(0,0,255);
  box(5);
  popMatrix();
}

void displayInfos()
{
  stroke(0, 200, 200);
  line (0,20,sizeX,20); // monitor bar
  fill(255);
  textSize(14);
  camPos = new PVector(cam.getPosition()[0], cam.getPosition()[1], cam.getPosition()[2]);  
  //String xCamera = str(0); //String.format("%.2f",position[0] );
  //String yCamera =  str(0); // String.format("%.2f",position[1] );
  //String zCamera = str(7); //String.format("%.2f",position[2] );
  text ("Rotation:", sizeX*0.05,sizeY*0.1-20);
  text (" X: " + camPos.x + "°"  , sizeX*0.05,sizeY*0.1);
  text (" Y: " + camPos.y  + "°", sizeX*0.05,sizeY*0.1+20);
  text (" Z: "+ camPos.z +"°) ", sizeX*0.05,sizeY*0.1+40);
  fill(0, 0, 200);
  
  textSize(12);
  text ("Quadcopter Orientation by UserK ", sizeX*0.7,sizeY*0.95);
}


void keyPressed() 
{
  //if (key == CODED) 
  //{
    switch (key) 
    {
      case 's':
        translate(-20,0,0);
        
        break;
      case 'a':
        translate(0,-20,0);
        break;
      case 'd':
        translate(0,20,0);
        break;
      case 'w':
        translate(20,0,0);
        break;
      case 'u':
        translate(0,0,20);
        break;
      case 'h':
        translate(0,0,-20);
        break;
      default:  
        break;
    }
  //}
}

