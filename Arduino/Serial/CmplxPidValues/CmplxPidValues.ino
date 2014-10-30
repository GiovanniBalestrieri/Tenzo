     byte byteRead;
     double num1, num2;
     double complNum,answer,counter;
     int numOfDec,optCount=0,letterCount=0;
     boolean mySwitch=false;
     boolean cmplx = false;
     char opt1[3];
     char opt2[3];
     char opt3[2];
  
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
             Serial.println();
             Serial.print("     opt1: ");
             Serial.print(opt1);
             Serial.println();
             Serial.print("     opt2: ");
             Serial.print(opt2);
             Serial.println();
             Serial.print("     opt3: ");
             Serial.print(opt3);
             Serial.println();
             Serial.print("    NUMBER: ");
             Serial.print(complNum);
             opt1[0] = (char)0;
             opt2[0] = (char)0;
             opt3[0] = (char)0;
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
         
         if ((byteRead>=65 && byteRead<=90) || (byteRead>=97 && byteRead<=122))
         {         
           Serial.println();
           Serial.print("lettera: ");
           Serial.print(byteRead);
           Serial.print("   ");
           if (cmplx)
           {
            // Listen for a capital letter or normal
            if (optCount==1 && letterCount<=3)
            {
             opt1[letterCount] = byteRead;
             Serial.print("letterCount: ");
             Serial.print(letterCount);
             Serial.print("   opt1: ");
             Serial.print(opt1);
            }
            else if (optCount==2 && letterCount<=3)
            {
             opt2[letterCount] = byteRead;
             Serial.print("letterCount: ");
             Serial.print(letterCount);
             Serial.print("   opt2: ");
             Serial.print(opt2);
            }
            else if (optCount==3 && letterCount<=2)
            {
             opt3[letterCount] = byteRead;
             Serial.print("letterCount: ");
             Serial.print(letterCount);
             Serial.print("   opt3: ");
             Serial.print(opt3);
            }
            letterCount++;
           }
         }
         //listen for numbers between 0-9
         if(byteRead>47 && byteRead<58)
         {
            //number found
            if (cmplx)
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
      
        if (byteRead==46)
        {
            mySwitch=true;
        }
     }
   }
