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
#define NUMPIXELS 160 // Popular NeoPixel ring size

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

#define MAX_INIT_REGS 32

class WAvrProg {
    public:
        uint8_t  m_prog[128 + 64];
        uint32_t m_init_regs[32];
        uint8_t  m_num_pixels;
        uint8_t  m_pix_offs;

        WAvrProg()
        {
            for (int i = 0; i < sizeof(m_prog); i++)
                m_prog[i] = 0;
            m_num_pixels = 0;
            m_pix_offs   = 0;
        }

        void set_reg_hsv(uint8_t idx, uint16_t h, uint8_t s, uint8_t v)
        {
            m_init_regs[idx] = pixels.ColorHSV(h, s, v);
        }
};

WAvrProg PROG;

#define MAX_RAND_COUNTERS 16
#define MAX_GREGS 16
#define MAX_REGS (MAX_GREGS + MAX_INIT_REGS)
#define LAST_REG (MAX_REGS - 1)

//#define REG_TIME_S  (LAST_REG)
//#define REG_PIXEL_F (LAST_REG - 1)
//#define REG_FRAME   (LAST_REG - 2)
//#define REG_TIME_F  (LAST_REG - 3)

class WAvrVMContext {
    public:
        uint32_t  m_regs[MAX_REGS];
        uint32_t  m_rand_counters[MAX_RAND_COUNTERS];
        float     m_slop_regs[MAX_RAND_COUNTERS];
        float     m_slop_incs[MAX_RAND_COUNTERS];

        WAvrVMContext()
        {
            for (int i = 0; i < MAX_REGS; i++)
                m_regs[i] = 0;

            for (int i = 0; i < MAX_RAND_COUNTERS; i++)
                m_rand_counters[i] = 0;

            for (int i = 0; i < MAX_RAND_COUNTERS; i++)
                m_slop_regs[i] = 0.0;

            for (int i = 0; i < MAX_RAND_COUNTERS; i++)
                m_slop_incs[i] = 0.0;
        }

        void load_prog_regs()
        {
            for (int i = MAX_GREGS; i < MAX_REGS; i++)
            {
                m_regs[i] = PROG.m_init_regs[i - MAX_GREGS];
            }
        }
};

WAvrVMContext CTX;


#define FR_SET(x, v) *((float *) &(x)) = (v);
#define FR_GET(x)    (*((float *) &(x)))

#define SV_REG(x)   do { regs[*(prog + 1)] = (x);      } while(0)
#define SV_F_REG(x) do { FR_SET(regs[*(prog + 1)], x); } while(0)

#define LD_REGS0_1() \
    do { \
        a = regs[*(prog + 1)]; \
    } while(0)

#define LD_REGS0_3() \
    do { \
        a = regs[*(prog + 1)]; \
        b = regs[*(prog + 2)]; \
        c = regs[*(prog + 3)]; \
    } while(0)

#define LD_REGS1() \
    do { \
        a = regs[*(prog + 2)]; \
    } while(0)

#define LD_REGS2() \
    do { \
        a = regs[*(prog + 2)]; \
        b = regs[*(prog + 3)]; \
    } while(0)

#define LD_PROG3() \
    do { \
        a = *(prog + 1); \
        b = *(prog + 2); \
        c = *(prog + 3); \
    } while(0)

#define LD_PROG_O_2() \
    do { \
        a = *(prog + 2); \
        b = *(prog + 3); \
    } while(0)

#define F_O_2() (((float) ((a << 8) | b)) / 65535.0)


#define OP_WIDTH 4
#define SKIP_PROG prog = prog + OP_WIDTH;

#define OP_SET      0x01
#define OP_MOV      0x02
#define OP_LDREG    0x03
#define OP_IMAP     0x04
#define OP_FMAP     0x05

#define OP_SLOPE    0x08
#define OP_RAND     0x09
#define OP_TPHASE   0x0A

#define OP_ADD      0x30
#define OP_ADDF     0x31
#define OP_MUL      0x32
#define OP_MULF     0x33
#define OP_DIV      0x34
#define OP_DIVF     0x35
#define OP_SUB      0x36
#define OP_SUBF     0x37

#define OP_FMOD     0x38
#define OP_FMODF    0x39

#define OP_SIN      0x40

#define OP_SET_RGB  0x10
#define OP_SET_HSV  0x11
#define OP_HSV      0x12
#define OP_RGB      0x13
#define OP_PIX_HSV  0x20
#define OP_PIX_RGB  0x21
#define OP_PIX      0x22
#define OP_PIX_N    0x23

#define OP_RET      0xFF

uint8_t  *init_prog = &PROG.m_prog[0];
uint32_t *regs = &CTX.m_regs[0];

uint16_t counter = 0;
uint32_t counter2 = 0;

void exec_prog() {
    uint8_t *prog = init_prog;

    bool prog_runs = true;

    uint32_t a = 0, b = 0, c = 0, d = 0;
    float ftmp = 0.0, ftmp2 = 0.0;

    uint8_t pixel_idx = PROG.m_pix_offs;

    uint32_t cur_time = millis();

    while (prog_runs)
    {
        switch (*prog)
        {
            case OP_SET:
                LD_PROG_O_2();
                SV_F_REG(F_O_2());
                break;

            case OP_MOV:
                LD_REGS1();
                SV_REG(a);
                break;

            case OP_RAND:
                LD_PROG_O_2();

                d = (a & 0xF0) >> 4;
                c = cur_time - CTX.m_rand_counters[d];

                // calculate the recalc-time:
                b *= 50;
                b *= (a & 0x0F) + 1;

                ftmp = CTX.m_slop_regs[d];
                ftmp += ((float) c) * CTX.m_slop_incs[d];

                if (ftmp > 1.0) ftmp = 1.0;
                if (ftmp < 0.0) ftmp = 0.0;

                if (c > b)
                {
                    CTX.m_slop_regs[d] = ftmp;

                    CTX.m_rand_counters[d] = cur_time;

                    ftmp2 = ((float) random(0, 65535)) / 65535.0;

                    ftmp2 = ftmp2 - CTX.m_slop_regs[d];
                    ftmp2 /= b;

                    CTX.m_slop_incs[d] = ftmp2;
                    c = b; // set c for the inc-multiplication below
                }

                // if (counter++ % 10 == 0)
                // {
                //     Serial.print(d);
                //     Serial.print(": ");
                //     Serial.println(ftmp);
                // }

                SV_F_REG(ftmp);
                break;

            case OP_ADD:
                LD_REGS2();
                SV_F_REG(FR_GET(a) + FR_GET(b));
                break;

            case OP_ADDF:
                LD_PROG_O_2();
                ftmp = F_O_2();
                LD_REGS0_1();

                SV_F_REG(FR_GET(a) + ftmp);
                break;

            case OP_MUL:
                LD_REGS2();
                SV_F_REG(FR_GET(a) * FR_GET(b));
                break;

            case OP_MULF:
                LD_PROG_O_2();
                ftmp = F_O_2();
                LD_REGS0_1();

                SV_F_REG(FR_GET(a) * ftmp);
                break;

            case OP_FMOD:
                LD_REGS2();

                if (abs(FR_GET(b)) > 0.00001)
                {
                    SV_F_REG(
                        FR_GET(a)
                        - floor(FR_GET(a) / FR_GET(b))
                          * FR_GET(b));
                }
                else
                {
                    SV_F_REG(0.0);
                }
                break;

            case OP_FMODF:
                LD_PROG_O_2();
                ftmp = F_O_2();
                LD_REGS0_1();

                if (abs(ftmp) > 0.00001)
                {
                    SV_F_REG(
                        FR_GET(a)
                        - floor(FR_GET(a) / ftmp)
                          * ftmp);
                }
                else
                {
                    SV_F_REG(0.0);
                }
                break;

            case OP_SIN:
                LD_REGS1();
                SV_F_REG((sin(FR_GET(a) * TWO_PI) + 1.0) * 0.5);
                break;

            case OP_SUB:
                LD_REGS2();
                SV_F_REG(FR_GET(a) - FR_GET(b));
                break;

            case OP_SUBF:
                LD_PROG_O_2();
                ftmp = F_O_2();
                LD_REGS0_1();

                SV_F_REG(FR_GET(a) - ftmp);
                break;

            case OP_DIV:
                LD_REGS2();
                if (abs(FR_GET(b)) > 0.00001)
                {
                    SV_F_REG(FR_GET(a) / FR_GET(b));
                }
                else
                {
                    SV_F_REG(0.0);
                }
                break;

            case OP_DIVF:
                LD_PROG_O_2();
                ftmp = F_O_2();
                LD_REGS0_1();

                if (abs(ftmp) > 0.00001)
                {
                    SV_F_REG(FR_GET(a) / ftmp);
                }
                else
                {
                    SV_F_REG(0.0);
                }
                break;

            case OP_TPHASE:
                LD_PROG_O_2();
                a = (uint32_t) ((a << 8) | b);
                SV_F_REG(
                    (((float) (cur_time % a))
                     / ((float) a)));
                break;

            case OP_RGB:
                LD_REGS0_3();
                regs[0] =
                    pixels.Color(
                        (uint8_t) (FR_GET(a) * 255.0),
                        (uint8_t) (FR_GET(b) * 255.0),
                        (uint8_t) (FR_GET(c) * 255.0));
                break;

            case OP_HSV:
                LD_REGS0_3();

                regs[0] =
                    pixels.ColorHSV(
                        (uint16_t) (FR_GET(a) * 65535.0),
                        (uint8_t)  (FR_GET(b) * 255.0),
                        (uint8_t)  (FR_GET(c) * 255.0));
                break;

            case OP_IMAP:
                LD_PROG_O_2();
                c = a; // min
                d = b; // max
                LD_REGS0_1();
                SV_REG((int) round(((float) c) + FR_GET(a) * ((float) (d - c))));
                break;

            case OP_FMAP:
                LD_REGS0_3();
                ftmp = FR_GET(b);
                SV_REG((int) round(b + FR_GET(a) * (c - b)));
                break;

            case OP_LDREG:
                LD_REGS1();
                SV_REG(regs[a % MAX_REGS]);
                break;

            case OP_SET_RGB:
                LD_PROG3();

                regs[0] = pixels.Color(a, b, c);
                break;

            case OP_SET_HSV:
                LD_PROG3();

                regs[0] =
                    pixels.ColorHSV(
                        (uint16_t) ((((float) a) * 65535.0) / 255.0),
                        b,
                        c);
                break;

            case OP_PIX:
                LD_REGS0_1();

                pixels.setPixelColor(pixel_idx, pixels.gamma32(a));
                pixel_idx++;
                // pixels.setPixelColor(a, pixels.gamma32(pixels.Color(0xFF, 0x00, 0x00)));
                break;

            case OP_PIX_N:
                LD_REGS0_1();
                b = *(prog + 2);

                for (int i = pixel_idx;
                     i < (pixel_idx + b)
                     && i < PROG.m_num_pixels;
                     i++)
                    pixels.setPixelColor(i, pixels.gamma32(a));
                pixel_idx += b;
                break;

            case OP_PIX_HSV:
                LD_REGS0_3();

                pixels.setPixelColor(
                    pixel_idx,
                    pixels.gamma32(pixels.ColorHSV(
                        (uint16_t) (FR_GET(a) * 65535.0),
                        (uint8_t)  (FR_GET(b) * 255.0),
                        (uint8_t)  (FR_GET(c) * 255.0))));
                break;

            case OP_PIX_RGB:
                LD_REGS0_3();
                pixels.setPixelColor(
                    pixel_idx,
                    pixels.gamma32(pixels.Color(
                        (uint8_t) (FR_GET(a) * 255.0),
                        (uint8_t) (FR_GET(b) * 255.0),
                        (uint8_t) (FR_GET(c) * 255.0))));
                break;

            case OP_RET:
                prog_runs = pixel_idx < PROG.m_num_pixels;
                prog = init_prog;
                prog = prog - OP_WIDTH;
                break;
            default:
                break;
        }

        SKIP_PROG;
    }
}

#define SERIAL_BUFFER_SIZE 20
char SERIAL_BUFFER[21];
int serial_buf_ptr = 0;
int serial_rd_ptr = 0;

void read_serial_buffer()
{
    while (true)
    {
        char c = Serial.read();

        if (c == -1)
        {
            delay(50);
            continue;
        }

        // Serial.print("[");
        // Serial.print((int) c);
        // Serial.println("]");

        if (serial_buf_ptr <= 0
            && (c == '\r' || c == '\n' || c == ' '))
            continue;

        if (c == '\r' || c == '\n')
        {
            SERIAL_BUFFER[serial_buf_ptr] = '\0';
            parse_serial_buffer();
            serial_buf_ptr = 0;
            return;
        }

        if (serial_buf_ptr >= SERIAL_BUFFER_SIZE)
            serial_buf_ptr = 0;

        SERIAL_BUFFER[serial_buf_ptr++] = c;
    }
}

bool test_next_str(const char *test, int &len)
{
    // <= is ok, because SERIAL_BUFFER is one byte bigger than SERIAL_BUFFER_SIZE
    for (int i = serial_rd_ptr; i <= SERIAL_BUFFER_SIZE; i++)
    {
        int tst_c = test[i - serial_rd_ptr];
        char c = SERIAL_BUFFER[i];
//        Serial.print(i);
//        Serial.print(":");
//        Serial.print(c);
//        Serial.print(" ");
//        Serial.print((char) tst_c);
//        Serial.print(" ");
//        Serial.print(serial_rd_ptr);
//        Serial.println();

        if (isWhitespace(c))
            return false;

        if (tst_c == '\0')
        {
            len = i - serial_rd_ptr;
            serial_rd_ptr += len;
            return true;
        }
        else if (tst_c != c)
            return false;
    }
}

void skip_ws()
{
    while (serial_rd_ptr < SERIAL_BUFFER_SIZE)
    {
        if (isWhitespace(SERIAL_BUFFER[serial_rd_ptr]))
        {
//            Serial.println("SKIPWS");
            serial_rd_ptr++;
        }
        else
            return;
    }
}

void parse_serial_buffer()
{
    serial_rd_ptr = 0;

    skip_ws();

    Serial.print("BUF[");
    Serial.print(SERIAL_BUFFER);
    Serial.println("]");
    int len = 0;
    if (test_next_str("PROG", len))
    {
        Serial.println("FOUND!");
        Serial.println(len);
    }
}

void setup() {
    // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
    // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
#endif
    // END of Trinket-specific code.

    Serial.begin(9600);

    // INITIALIZE NeoPixel strip object (REQUIRED)
    pixels.begin();
    pixels.show();            // Turn OFF all pixels ASAP
    pixels.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    int pc = 0;

#define NEW_OP(op, a1, a2, a3) \
    PROG.m_prog[pc++] = op; \
    PROG.m_prog[pc++] = a1; \
    PROG.m_prog[pc++] = a2; \
    PROG.m_prog[pc++] = a3; \

//    NEW_OP(OP_SET_RGB, 0xFF, 0x00, 0xFF);
//    NEW_OP(OP_MOV,     0x08, 0x00, 0x00);
//    NEW_OP(OP_PIX_N,   0x08, 0x05, 0x00);
//    NEW_OP(OP_PIX_N,   0x09, 0x05, 0x00);
//    NEW_OP(OP_PIX_N,   0x0A, 0x05, 0x00);
//    NEW_OP(OP_MULF,    0x04, 0x60, 0x00);
//    NEW_OP(OP_ADDF,    0x04, 0x30, 0x00);
//    NEW_OP(OP_HSV,     0x04, 0x05, 0x05);
//    NEW_OP(OP_PIX_N,   0x00, 0x03, 0x00);
//    NEW_OP(OP_SET_HSV, 0x7F, 0x7F, 0xFF);
//    //NEW_OP(OP_RAND,    0x07, 0x10, 0x40);

    NEW_OP(OP_SET,     0x05, 0xFF, 0xFF);
    NEW_OP(OP_SET,     0x04, 0x7F, 0xFF);
    NEW_OP(OP_RAND,    0x06, 0x10, 0x3F);
    NEW_OP(OP_HSV,     0x06, 0x04, 0x05);
    NEW_OP(OP_PIX_N,   0x00, 0x05, 0x00);
    NEW_OP(OP_RET,     0x00, 0x00, 0x00);

///    NEW_OP(OP_TPHASE,  0x04, 0x1F, 0x00);
///    NEW_OP(OP_SIN,     0x04, 0x04, 0x00);
///    NEW_OP(OP_IMAP,    0x04, 0x10, 0x12);
///    NEW_OP(OP_LDREG,   0x00, 0x04, 0x00);
///    NEW_OP(OP_PIX_N,   0x00, 0x10, 0x00);
///    NEW_OP(OP_RET,     0x00, 0x00, 0x00);

    PROG.m_num_pixels = 160;
    PROG.m_pix_offs   = 1;

    PROG.set_reg_hsv(0, 0x1FFF, 0xFF, 0xFF);
    PROG.set_reg_hsv(1, 0x3FFF, 0xFF, 0xFF);
    PROG.set_reg_hsv(2, 0x07FF, 0xFF, 0xFF);

    CTX.load_prog_regs();
}

void loop() {
    pixels.clear();
    unsigned long start_t = micros();
    exec_prog();
    pixels.show();

    unsigned long end_t = micros();

    if (Serial.available() > 0)
    {
        char c  = Serial.read();
        if (c == '!')
        {
            Serial.println("BEGIN");
            Serial.flush();
        }

        read_serial_buffer();

        Serial.println("END");
        Serial.flush();
    }

    if (counter % 60 == 0)
    {
        Serial.print("frame: ");
        Serial.println(end_t - start_t);
    }

    counter++;
}
