void menu_utama()
{byte layar=0,jumlahmenu=4,awal=0,akhir=4,baris,tunggu;
 modeSet=true;
 while(layar<20)
 {BP32.update();processControllers();
  if(layar<awal){awal=layar;akhir=layar+jumlahmenu;}
  if(layar>akhir){akhir=layar;awal=akhir-jumlahmenu;}
  baris=1;
  for(unsigned char a=awal;a<=akhir;a++)
  {tampiltext(3,baris,1,"");baris=baris+10;
   switch(a)
   {case 0 : display.print("Speed Normal:"+String(Speed));break;
    case 1 : display.print("Speed Booster:"+String(Boost));break;
    case 2 : display.print("PWM freq:"+String(frekuensiPWM)+"Hz");break;
    case 3 : display.print("Reset connection?");   break;
    case 4 : display.print("Reset Parameter?");        break; 
    case 5 : if(statGyro){display.print("GYRO Status:ON");}
             else{display.print("GYRO Status:OFF");}
    break;
    case 6 : display.print("KP gyro:"+String(Kp,2));break;
    case 7 : display.print("KI gyro:"+String(Ki,2));break;
    case 8 : display.print("KD gyro:"+String(Kd,2));break;
    case 9 : display.print("Sampling PID:"+String(Tsamp)+"ms");break;
    case 10: display.print("PID Enter At:"+String(enterPID)+"ms");break;
    case 11: display.print("PID Zero At:"+String(autoOff)+"ms");break;
    case 12: display.print("Calibrate Gyro?");break;
   }
   if(a==layar){display.fillRect(1,((a-awal)*10),116,9,SSD1306_INVERSE);}
  }
  background(12,layar);
  display.display();display.clearDisplay();
  
  layar=SWITCH(0,0,12,layar,1);
  switch(layar)
  {case 0 : Speed         = SWITCH(1,0,255,Speed,1);          break;
   case 1 : Boost         = SWITCH(1,0,255,Boost,1);          break;
   case 2 : frekuensiPWM  = SWITCH(1,0,5000,frekuensiPWM,25); break;
   case 5 : statGyro      = SWITCH(1,0,1,statGyro,1);         break;
   case 6 : Kp            = SWITCH(1,0,200,Kp,0.01);          break;
   case 7 : Ki            = SWITCH(1,0,200,Ki,0.01);          break;
   case 8 : Kd            = SWITCH(1,0,200,Kd,0.01);          break;
   case 9 : Tsamp         = SWITCH(1,0,50,Tsamp,1);           break;
   case 10: enterPID      = SWITCH(1,0,2000,enterPID,1);     break;
   case 11: autoOff       = SWITCH(1,0,9999,autoOff,1);      break;
   case 4 : if(silang)
            {while(silang){BP32.update();processControllers();}
             pref.putUChar("Speed",130);
             pref.putUChar("Tsamp",4);
             pref.putUChar("Boost",255);
             pref.putFloat("Kp",3.0);
             pref.putFloat("Ki",0.0);
             pref.putFloat("Kd",12.0);
             pref.putBool("statGyro",true);
             pref.putInt("frekuensiPWM",675);
             pref.putInt("autoOff",1000);
             pref.putInt("enterPID",300); 
             ledcSetup(4,675,8);ledcAttachPin(ML1,4);
             ledcSetup(5,675,8);ledcAttachPin(ML2,5);
             ledcSetup(6,675,8);ledcAttachPin(MR1,6);
             ledcSetup(7,675,8);ledcAttachPin(MR2,7);
             display.clearDisplay();
             background(12,layar);
             baca_preferences();
             tampiltext(0, 20, 1, "Memory Restored!");
             display.display();
             delay(2000);
            }
            break;
   case 3 : if(silang)
            {while(silang){BP32.update(); processControllers();}
             pref.remove("macStik");
             BP32.forgetBluetoothKeys();
             display.clearDisplay();
             tampiltext(0, 20, 1, "Memory Cleared!");
             tampiltext(0, 35, 1, "Restarting...");
             display.display();
             delay(2000);
             esp_restart();
            }                
            break;
   case 12: if(silang)
            {while(silang){BP32.update();processControllers();}
             pushSW=0;tunggu=1;
             background(12,layar);
             tampiltext(0,0,1,"Please don't move");
             tampiltext(0,8,1,"the BOT!");
             display.display();display.clearDisplay();
             while(pushSW<3000){delay(1);if(IN!=0){tunggu=0;pushSW=3000;}}
             if(tunggu==1)
             {pushSW=0;
              while(pushSW<3000)
              {background(12,layar);
               tampiltext(0,0,1,"Calibrating in.."+String(3-(pushSW/1000)));
               display.display();display.clearDisplay();
               if(IN!=0){tunggu=0;pushSW=3000;} 
              }
             }
             if(tunggu==1)
             {calibGyro=true;
              background(12,layar);
              tampiltext(0,0,1,"Calibrating...");
              tampiltext(0,10,1,"PLEASE WAIT!");
              display.display();display.clearDisplay();
              float sum = 0;
              for (int i=0;i<200;i++) 
              {sensors_event_t a, g, temp;
               mpu.getEvent(&a, &g, &temp);
               sum += g.gyro.z;
               delay(5);
              }
              gyroZOffset = sum / 200.0;
              pref.putFloat("gyroZOffset",gyroZOffset);
              calibGyro=false;
              background(12,layar);tampiltext(0,0,1,"DONE !");
              display.display();display.clearDisplay();
              delay(1000);sudutZ=0;
             }
            }
            break; 
  }
  if(bulet)
  {while(bulet){BP32.update();processControllers();}
   layar=20;
   pref.putUChar("Speed",Speed);
   pref.putUChar("Tsamp",Tsamp);
   pref.putUChar("Boost",Boost);
   pref.putFloat("Kp",Kp);
   pref.putFloat("Ki",Ki);
   pref.putFloat("Kd",Kd);
   pref.putBool("statGyro",statGyro);
   pref.putInt("frekuensiPWM",frekuensiPWM);
   pref.putInt("autoOff",autoOff);
   pref.putInt("enterPID",enterPID); 
   ledcSetup(4,frekuensiPWM,8);ledcAttachPin(ML1,4);
   ledcSetup(5,frekuensiPWM,8);ledcAttachPin(ML2,5);
   ledcSetup(6,frekuensiPWM,8);ledcAttachPin(MR1,6);
   ledcSetup(7,frekuensiPWM,8);ledcAttachPin(MR2,7);
  }
 }
 modeSet=false; 
}
