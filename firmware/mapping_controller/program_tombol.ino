byte modeSW[2];
float SWITCH(byte x,float batasRendah,float batasTinggi,float input,float kelipatan)
{int masok=-1;
 if(IN==0){masok=0;}
 else{if(x==0)
      {if(atas){masok=1;}
       if(bawah){masok=2;}
      }
      else
      {if(kiri){masok=1;}
       if(kanan){masok=2;}
      }
     }

 switch(masok)
 {case 0 : modeSW[0]=modeSW[1]=pushSW=0;
  break;
  case 1 : if(pushSW<100)
           {pushSW=100;
            if(input>batasRendah){input-=kelipatan;}else{input=batasTinggi;}
           }
           else
           {if(modeSW[x]<5){if(pushSW>300){pushSW=0;modeSW[x]++;}}
            else{if(pushSW>120){pushSW=0;}}
           }
  break;
  case 2 : if(pushSW<100)
           {pushSW=100;
            if(input<batasTinggi){input+=kelipatan;}else{input=batasRendah;}
           }
           else
           {if(modeSW[x]<5){if(pushSW>300){pushSW=0;modeSW[x]++;}}
            else{if(pushSW>120){pushSW=0;}}
           }
  break;
 }
 return input;
}
