   /**
    *   Expected Serial command:
    *
    *   X,[0-9],[0-9],[0-9],[1.2f],X
    **/   
   byte byteRead;
   double num1, num2;
   double complNum,answer,counter;
   int numOfDec,optCount=0,letterCount=0;
   boolean mySwitch=false;
   boolean cmplx = false;
   int opt1,opt2,opt3;
   char* options1[4] = {"Roll","pitch","Yaw","Altitude"};
   char* options2[2]  = {"Conservative","Aggressive"};
   char* options3[3]  = {"proportional","Derivative","Integral"};
   void setup() 
   {                
     /* Turn the Serial Protocol ON and 
     initialise num1 and num2 variables.*/
     Serial.begin(9600);
     num1=0;
     num2=0;
     complNum=0;
     counter=1;
     numOfDec=0;
   }

   void loop() 
   {
     /*  check if data has been sent from the computer: */
     while (Serial.available()) 
     {
       /* read the most recent byte */
       byteRead = Serial.read();
        
       if (byteRead == 'X')
       {
         if (!cmplx)
         {
           // begin of the string
           cmplx =true;
         }
         else
         {
           // end of the string - reset values
           cmplx=false;
           optCount=0;
           /* Create the double from num1 and num2 */
           complNum=num1+(num2/(counter));
           /* Reset the variables for the next round */
           Serial.print("     opt1: ");
           Serial.print(options1[opt1]); 
                     
           Serial.print("     opt2: ");
           Serial.print(options2[opt2]); 
                     
           Serial.print("     opt3: ");
           Serial.print(options3[opt3]); 
           
           
           Serial.print("    Value: ");
           Serial.print(complNum);
           num1=0;
           num2=0;
           complNum=0;
           counter=1;
           mySwitch=false;
           numOfDec=0;
         }
       }
       if (byteRead==44)
       {
         // Comma
         optCount++;
         Serial.println();
         Serial.print("virgola numero: ");
         Serial.println(optCount);
         letterCount = 0;
       }

       //listen for numbers between 0-9
       if(byteRead>47 && byteRead<58)
       {
          //number found
          if (cmplx)
          {
            if (optCount == 1)
            {
             opt1 = byteRead - 48; 
             Serial.print(opt1);
            }
            else if (optCount == 2)
            {
             opt2 = byteRead - 48; 
             Serial.print(opt2);
            }
            else if (optCount == 3)
            {
             opt3 = byteRead - 48; 
             Serial.print(opt3);
            }
            if (optCount == 4)
            {
              /* If mySwitch is true, then populate the num1 variable
              otherwise populate the num2 variable*/
              if(!mySwitch)
              {
                num1=(num1*10)+(byteRead-48);
              }
              else
              {
                num2=(num2*10)+(byteRead-48);           
                /* These counters are important */
                counter=counter*10;
                numOfDec++;
              }
            }
          }
       }
    
      if (byteRead==46)
      {
          mySwitch=true;
      }
   }
 }
