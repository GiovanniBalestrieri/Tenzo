void acquisisciMess() {

  int i;

  for(i=0;i<MAX_LEN;i++){
    data[i]=0;
  }

  Serial.println("Scrivi un messaggio e premi INVIO");

  while (Serial.available() == 0) {
    // Attendi che venga digitato un messaggio
  }

  if (Serial.available() > 0) {
    for (i = 0; i < MAX_LEN; i++) {
      data[i]=Serial.read();
    }
  }

  Serial.print("Messaggio in uscita: ");
  Serial.println((char*)data);

}

