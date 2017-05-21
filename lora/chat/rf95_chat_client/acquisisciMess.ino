void acquisisciMess() {
  
  if (Serial.available()) {

    check = 1;

    int i;
    char messaggio[MAX_LEN];
   
    for (i = 0; i < MAX_LEN; i++) {
      messaggio[i] = 0;
    }
  
    
  
    i = 0;
    while (Serial.available() > 0) {
       messaggio[i] = Serial.read();
       i++;
    }
  
    String payload(messaggio);
   
    payload.getBytes(data, payload.length());
  
    //Serial.print("Messaggio in uscita: ");
    //Serial.println((char*)data);

  } else {
    check= 0;
  }
}
