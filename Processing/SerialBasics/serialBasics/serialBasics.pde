import processing.serial.*;
Serial myPort;
String val="nada";

void setup()
{
  size(200,200);
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200); 
}

void draw()
{
  if ( myPort.available() > 0) 
  {  
    val = myPort.readStringUntil('\n'); 
    String[] q = splitTokens(val,",");
    println();
    println("Phi: " + q[0]);
    println("Theta: " + q[1]);
    println("Psi: " + q[2]);
    println();
  } 

  if (mousePressed == true) 
  {                           //if we clicked in the window
   myPort.write('A');         //send a 1
   println("A");   
  }
  /*else 
  {                           //otherwise
    myPort.write('B');          //send a 0
  } */  

}
