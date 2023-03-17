

#include <NeoPixelBus.h>

const uint16_t PixelCount = 60; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 4;  // make sure to set this to the correct pin, ignored for Esp8266
const uint8_t PixelPin2 = 3;  

const uint8_t buttonOnePin = 6;
const uint8_t buttonTwoPin = 5;


#define colorSaturation 85

// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip2(PixelCount, PixelPin2);

//NeoPixelBus<NeoRgbFeature, Neo400KbpsMethod> strip2(PixelCount, PixelPin2);

RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation);
RgbColor black(0);
//RgbColor yellow(85, 83, 7);
RgbColor yellow(78,72,17);
RgbColor purple(57, 7, 85);

/*
RgbColor red(colorSaturation, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor blue(0, 0, 255);
RgbColor white(colorSaturation);
RgbColor black(0);
RgbColor yellow(255, 251, 20);
RgbColor purple(173, 20, 255);
*/
int k = 0;
int thickness = 5; //How many pixels long the blue pixels are in pattern1 
int count = 0;

RgbColor ledArray[PixelCount];
RgbColor ledArray2[PixelCount];

void setup()
{
    pinMode(buttonOnePin, INPUT);
    pinMode(buttonTwoPin, INPUT);

    Serial.begin(115200);
    //while (!Serial); // wait for serial attach

    Serial.println();
    Serial.println("Initializing...");
    Serial.flush();

    // this resets all the neopixels to an off state
    strip.Begin();
    strip.Show();

    strip2.Begin();
    strip2.Show();


    Serial.println();
    Serial.println("Running...");
}


void loop()
{
  bool yellow_button = digitalRead(buttonOnePin);
  bool purple_button = digitalRead(buttonTwoPin);
  bool pattern1_button = yellow_button && purple_button;

   if (yellow_button && !pattern1_button) {
      for (int i = 0; i < PixelCount; i++) {
        ledArray[i] = yellow;
    }
   } else if (purple_button && !pattern1_button) {
      for (int i = 0; i < PixelCount; i++) {
        ledArray[i] = purple;
      } 
   } else if (pattern1_button){
      for (int i = 0; i < PixelCount; i++) {
        ledArray2[i] = yellow;
    }
        for(int l = 0+count; l < thickness+count; l++){
        ledArray2[l%PixelCount] = blue; 
      } 
        count++;
    
   } else {
     for (int o = 0; o < PixelCount; o++) {
       
       if (((o + k) % 2) == 0) {
         ledArray[o] = blue;
       } else {
         ledArray[o] = yellow;
       }
     }
   
     if (k == 0) {
       k = 1;
     } else {
       k = 0;
     }
   }
    Serial.print("Button one:");
    Serial.print(digitalRead(buttonOnePin));
    Serial.print("  Button two:");
    Serial.println(digitalRead(buttonTwoPin));

    


    for (int i = 0; i < PixelCount; i++) {
      if(!pattern1_button){
       strip.SetPixelColor(i, ledArray[i]);
       strip2.SetPixelColor(i, ledArray[i]);
      } else {
        strip.SetPixelColor(i,ledArray2[i]); 
        strip2.SetPixelColor(i,ledArray2[i]); 

      }
    }


    strip.Show();
    strip2.Show();

  if(pattern1_button){
    delay(200);
  } else {
    delay(400);
  }



}

