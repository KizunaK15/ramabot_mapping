void background(float jumlahmenu,float menuke)
{tampiltext(0,56,1," X:SELECT      O:BACK");
 display.fillRect(0,55,128,9,SSD1306_INVERSE);
 display.drawFastVLine(125,0,55,SSD1306_INVERSE);
 display.fillRect(123,39.0*(menuke/jumlahmenu),5,16,SSD1306_WHITE); 
}
