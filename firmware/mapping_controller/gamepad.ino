bool consolConnect=false;

void onConnectedController(ControllerPtr ctl) 
{if (myControllers[0] != nullptr){ctl->disconnect(); return;}
  
 auto properties = ctl->getProperties();
 const uint8_t* addr = properties.btaddr;
 
  bool memoriKosong = true;
  for(int i=0; i<6; i++){if(macTersimpan[i]!=0 && macTersimpan[i]!=255) {memoriKosong = false;break;}} 
  
  if (memoriKosong) 
  {pref.putBytes("macStik", addr, 6);
   for(byte x=0;x<6;x++){macTersimpan[x]=addr[x];}
   myControllers[0] = ctl;
   consolConnect = true;
   display.clearDisplay();
   tampiltext(3, 0, 1, "  CONSOL CONNECTED  ");
   display.drawFastHLine(0, 10, 128, SSD1306_WHITE); 
   tampiltext(0, 20, 1, "New MAC Address Saved");
   tampiltext(0, 32, 1, "MAC:");
    for(int i=0; i<6; i++) 
    {display.printf("%02X", macTersimpan[i]);
     if(i<5) display.print(":");
    }
  } 
  else 
  {bool cocok = true;
   for(int i=0; i<6;i++){if(addr[i]!=macTersimpan[i]){cocok = false;break;}}
   if(cocok)
   {myControllers[0]=ctl;
    consolConnect=true;display.clearDisplay();
    tampiltext(3, 0, 1, "  CONSOL CONNECTED  ");
    display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
    tampiltext(0, 22, 1, "Good luck playing!!");
    tampiltext(0, 32, 1, "MAC:");
    for(int i=0; i<6; i++) 
    {display.printf("%02X", addr[i]);
     if(i<5) display.print(":");
    }
    display.setCursor(0, 42);
    display.print(ctl->getModelName().c_str());
   } 
   else 
   {ctl->disconnect();myControllers[0]=nullptr;
    return;
   }
  }
  ledcWrite(3, 255);display.display();
  BP32.enableNewBluetoothConnections(false); 
  delay(3000); 
  display.clearDisplay();
}

void onDisconnectedController(ControllerPtr ctl) 
{if (myControllers[0] == ctl) 
 {myControllers[0] = nullptr;
  ledcWrite(3, 60);consolConnect=false;
  tampiltext(0, 0, 1, " CONSOL DISCONNECTED ");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  tampiltext(0, 22, 1, "Search Mac Address:");
  tampiltext(0,32,1,"");
  for(int i=0; i<6; i++) 
  {display.printf("%02X", macTersimpan[i]);
   if(i<5) display.print(":");
  }
  display.display();display.clearDisplay();
  delay(3000);
  BP32.enableNewBluetoothConnections(true);
 }
}

void dumpGamepad(ControllerPtr ctl) {
    IN = ctl->buttons() | (ctl->dpad() << 8) | (ctl->miscButtons() << 11);

    silang   = (IN & 0x01); 
    bulet    = (IN & 0x02);
    kotak    = (IN & 0x04);
    segitiga = (IN & 0x08);
    l1       = (IN & 0x10);
    r1       = (IN & 0x20);
    l2       = (IN & 0x40);
    r2       = (IN & 0x80);
    atas     = (IN & 0x100);
    bawah    = (IN & 0x200);
    kanan    = (IN & 0x400);
    kiri     = (IN & 0x800);
    SELECT   = (IN & 0x1000);
    START    = (IN & 0x2000);

    analogLX = ctl->axisX();
    analogLY = ctl->axisY();
    analogRX = ctl->axisRX();
    analogRY = ctl->axisRY();
}

void processGamepad(ControllerPtr ctl) {
  if (ctl->a()) {
    static int colorIdx = 0;
    switch (colorIdx % 3) {
      case 0: ctl->setColorLED(255,0,0); break;
      case 1: ctl->setColorLED(0,255,0); break;
      case 2: ctl->setColorLED(0,0,255); break;
    }
    colorIdx++;
  }

  if (ctl->b()) {
    static int led = 0;
    led++;
    ctl->setPlayerLEDs(led & 0x0F);
  }

  if (ctl->x()) {
    ctl->playDualRumble(0, 250, 0x80, 0x40);
  }

  dumpGamepad(ctl);
}

void processControllers() {
  ControllerPtr ctl = myControllers[0];
  if (ctl && ctl->isConnected() && ctl->hasData()) {
    if (ctl->isGamepad()) {
      processGamepad(ctl);
    } else {display.clearDisplay();tampiltext(0,0,1,"Unsupported controller");
      display.display();display.clearDisplay();delay(3000);
    }
  }
}

void connectGamepad() 
{bool memoriKosong = true;
 for(int i=0; i<6; i++){if(macTersimpan[i]!=0 && macTersimpan[i]!=255) {memoriKosong = false;break;}}

 if(!memoriKosong)
 {tampiltext(3,0,1,"  SEARCHING CONSOL  ");
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
  tampiltext(0, 20, 1, "Hold BOOT: New Pair");
  display.display();
  tampiltext(0, 30, 1, "");
 }
 while (!consolConnect) 
 {BP32.update();
  if((digitalRead(0)==LOW)||(memoriKosong)) 
  {pref.remove("macStik");
   for(byte x=0;x<6;x++){macTersimpan[x]=0;}
   BP32.forgetBluetoothKeys();
   BP32.enableNewBluetoothConnections(true);

   display.clearDisplay();
   tampiltext(0, 0, 1, "  MODE: NEW PAIRING  ");
   display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
   tampiltext(0, 20, 1, "Last MAC Address");
   tampiltext(0, 30, 1, "Deleted!");
   tampiltext(0, 40, 1, "Search new CONSOL!");
   tampiltext(0,50,1,"");
   display.display(); 
   while(!consolConnect) 
   {BP32.update();
    if(pushSW>1000){display.print(".");display.display();pushSW = 0;}
   }
   break;
  }

  if (pushSW > 1000){display.print(".");display.display();pushSW = 0;}
  delay(10);
 }
}
