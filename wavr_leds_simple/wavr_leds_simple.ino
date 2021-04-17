// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 46 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 100 // Time (in milliseconds) to pause between pixels


float fract(float x) {
  return x - int(x);
}

float mix(float a, float b, float t) {
  return a + (b - a) * t;
}

float step(float e, float x) {
  return x < e ? 0.0 : 1.0;
}

float* hsv2rgb(float h, float s, float b, float* rgb) {
  rgb[0] = b * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[1] = b * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[2] = b * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  return rgb;
}

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}


float hue = 0.0;
float offs = 0.0;
int mode = 0;

struct WAvrProg {
    unsigned char m_prog[128 + 64];
    unsigned char m_num_pixels;
    unsigned char m_pix_offs;

    WAvrProg()
    {
        for (int i = 0; i < sizeof(m_prog); i++)
            m_prog[i] = 0;
        m_num_pixels = 0;
        m_pix_offs   = 0;
    }
};

struct WAvrVMContext {
    unsigned char   m_regs[8 * 4];
    short           m_segment;
    short           m_pixel;
    float           m_pixel_f;
    short           m_frame;

    WAvrVMContext()
    {
        for (int i = 0; i < sizeof(m_regs); i++)
            m_regs[i] = 0;
        m_segment    = 0;
        m_pixel      = 0;
        m_pixel_f    = 0.0;
        m_frame      = 0;
    }
};

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  offs = 0.0;
  hue = 0.0;
  mode = 0;
  Serial.begin(115200);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.show();            // Turn OFF all pixels ASAP
  pixels.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop() {
  offs += 0.01;
  offs = fract(offs);
  float rgb[3] = { 0.0, 0.0, 0.0 };

  for (int i = 0; i < NUMPIXELS; i++) {
    float v = ((float) i) / ((float) NUMPIXELS);
    v = abs(fract(v + offs));
    hsv2rgb(v, 1.0, 1.0, rgb);
    pixels.setPixelColor(
      i,
      pixels.gamma32(
        pixels.Color(
          rgb[1] * 255.0,
          rgb[0] * 255.0,
          rgb[2] * 255.0)));
  }

  pixels.show();
  delay(1000 / 30);
}


/*
  #if 0
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'x') {
      mode = 1;
      Serial.print("ok x\n");
    } else if (c == 'y') {
      mode = 2;
      Serial.print("ok y\n");
    }
  }

  hue += 0.1;
  hue = fract(hue);
  pixels.clear(); // Set all pixel colors to 'off'
  #endif
*/
//      Serial.print(offs);
//      Serial.println("");
