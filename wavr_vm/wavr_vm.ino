#include <SoftwareSerial.h>

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
#define NUMPIXELS 300 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 100 // Time (in milliseconds) to pause between pixels

SoftwareSerial softSerial(10, 11); // RX, TX

float fract(float x) {
  return x - int(x);
}

float mix(float a, float b, float t) {
  return a + (b - a) * t;
}

float step(float e, float x) {
  return x < e ? 0.0 : 1.0;
}

#define MAX_INIT_REGS 8

class WAvrProg {
    public:
        uint8_t  m_prog[128 + 64];
        uint32_t m_init_regs[MAX_INIT_REGS];
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

#define MAX_RAND_COUNTERS 8
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

// Registers
// - REG[00-15]: General Purpose Registers
// - REG[16-47]: Initializable HSV Color Registers (Colors)

#define OP_SET      0x01 // a = float(b | c)
#define OP_MOV      0x02 // a = b
#define OP_LDREG    0x03 // a = REG[b]
#define OP_IMAP     0x04 // a = map_to_int_range(x: float(REG[a]), min: b, max: c)
#define OP_FMAP     0x05 // a = map_to_flt_range(x: float(REG[a]), min: float(REG[b]), max: float(REG[c]))

// b & 0xF0: Rand Counter Index [0-15]
// c       : Recalc-Time (ms) = c * 50 * (b & 0x0F) + 1
// a = float(incremental adjustment within recalc-time to rand 0.0 to 1.0)
#define OP_RAND     0x09
// (b << 8 | c): Phase length in ms
// a = float(fractional phase)
#define OP_TPHASE   0x0A

#define OP_SET_RGB  0x10 // 0 = rgb(a, b, c)
#define OP_SET_HSV  0x11 // 0 = hsv(a, b, c)
#define OP_HSV      0x12 // 0 = hsv(float(REG[a]), float(REG[b]), float(REG[c]))
#define OP_RGB      0x13 // 0 = rgb(float(REG[a]), float(REG[b]), float(REG[c]))

// P means: write pixel and advance to the next pixel
#define OP_PIX_HSV  0x20 // P = hsv(float(REG[a]), float(REG[b]), float(REG[c]))
#define OP_PIX_RGB  0x21 // P = rgb(float(REG[a]), float(REG[b]), float(REG[c]))
#define OP_PIX      0x22 // P = color(REG[a])
#define OP_PIX_N    0x23 // b times(P = color(REG[a]))

#define OP_ADD      0x30 // a = REG[b] + REG[c]
#define OP_ADDF     0x31 // a = float(float(REG[b]) + float(REG[c]))
#define OP_MUL      0x32 // a = REG[b] * REG[c]
#define OP_MULF     0x33 // a = float(float(REG[b]) * float(REG[c]))
#define OP_DIV      0x34 // a = REG[b] / REG[c]
#define OP_DIVF     0x35 // a = float(float(REG[b]) / float(REG[c]))
#define OP_SUB      0x36 // a = REG[b] - REG[c]
#define OP_SUBF     0x37 // a = float(float(REG[b]) - float(REG[c]))

#define OP_FMOD     0x38 // a = float(REG[b]) % float(REG[c])
#define OP_FMODF    0x39 // a = float(REG[a]) % float(b | c)

#define OP_SIN      0x40 // a = float(sin(float(REG[b])))

#define OP_RET      0xFF // Restart until all pixels have been written, or end cycle

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
                ftmp  = FR_GET(b);
                ftmp2 = FR_GET(c);
                SV_F_REG(ftmp + FR_GET(a) * (ftmp2 - ftmp));
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

/*

Data Protocol:

Both parties ignore CR LF.

Handshake:

    Client sends:   '!'
    Server answers: 'S'
    If Server receives a '.', it sends an 'E'.
    Server any other char received: '?'

    - Server repeats 'S' if another '!' is received
    - Client needs to repeat '!' if no 'S' is received in 1 second
    or a '?' is received.


    Prog line:

    Client sends:    <prog index> <command> <arg1> <arg2> <arg3> <bcc> '#'
    Server answers:  '@'
    Server bad bcc response: '?'

    - If server receives invalid bcc, it sends a '?'
    - If server receives a corrent bcc, it sends a '@'
    - If client does not receive an '@' within 1 second
    or receives a '?', it retransmit the command.

    End of input:

    Client sends:   '.'
    Server answers: 'E'
    Server any other char received: '?'

    - If client does not receive 'E' or a '?' it repeats '.'
    until it makes sure the server understood.
    - Server needs to send an 'E' also if it receives an '.'
    without a prior '!'.

 */
void sendAtCommand(const char *cmd, char postFix, bool recvResponse)
{
    char buf[20];
    char rcv[20];
    char rcv2[20];
    size_t l = 0, l2 = 0;

    int i = 0;
    while (cmd[i] != '\0')
    {
        buf[i] = cmd[i];
        i++;
    }

    if (postFix > 0)
        buf[i++] = postFix;
    buf[i++] = '\r';
    buf[i++] = '\n';

    softSerial.write(buf, i);
    softSerial.flush();

    l = softSerial.readBytesUntil('\n', rcv, 16);
    if (l > 0 && rcv[l - 1] == '\r')
        l--;

    if (recvResponse)
    {
        l2 = softSerial.readBytesUntil('\n', rcv2, 16);
        if (l2 > 0 && rcv2[l2 - 1] == '\r')
            l2--;
    }

    Serial.print("H:");
    Serial.write(buf, i - 2);
    Serial.print("> ");
    Serial.flush();

    Serial.write(rcv, l);
    Serial.println("");
    Serial.flush();

    if (l2 > 0)
    {
        Serial.print("H>");
        Serial.write(rcv2, l2);
        Serial.println("");
        Serial.flush();
    }
}

char waitNextCharSerial()
{
    while (!Serial.available())
        delay(25);
    return Serial.read();
}

void skip_ws(uint8_t &bcc)
{
    while (serial_rd_ptr < SERIAL_BUFFER_SIZE)
    {
        if (isWhitespace(SERIAL_BUFFER[serial_rd_ptr]))
        {
            //            Serial.println("SKIPWS");
            bcc ^= (uint8_t) SERIAL_BUFFER[serial_rd_ptr];
            serial_rd_ptr++;
        }
        else
            return;
    }
}

bool test_next_str(const char *test, int &len, uint8_t &bcc)
{
    uint8_t cur_bcc = 0;

    // <= is ok, because SERIAL_BUFFER is one byte bigger than SERIAL_BUFFER_SIZE
    for (int i = serial_rd_ptr; i <= SERIAL_BUFFER_SIZE; i++)
    {
        int tst_c = test[i - serial_rd_ptr];
        char c = SERIAL_BUFFER[i];

        cur_bcc ^= (uint8_t) c;
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
            bcc ^= cur_bcc;
            len = i - serial_rd_ptr;
            serial_rd_ptr += len;
            return true;
        }
        else if (tst_c != c)
            return false;
    }
}

struct ParseCmd
{
    uint8_t     prog_addr;
    uint8_t     command;
    uint8_t     arg1;
    uint8_t     arg2;
    uint8_t     arg3;
};

uint8_t parse_hex_byte(bool &ok, uint8_t &bcc)
{
    skip_ws(bcc);

    uint8_t b           = 0;
    bool    high_nibble = true;

    // <= is ok, because SERIAL_BUFFER is one byte bigger than SERIAL_BUFFER_SIZE
    for (int i = serial_rd_ptr; i <= SERIAL_BUFFER_SIZE; i++)
    {
        char c = SERIAL_BUFFER[i];
        if (c >= '0' && c <= '9')
            b |= c - '0';
        else if (c >= 'a' && c <= 'f')
            b |= c - 'a';
        else if (c >= 'A' && c <= 'F')
            b |= c - 'A';
        else
        {
            ok = false;
        }

        bcc ^= (uint8_t) c;

        if (high_nibble)
        {
            b = b << 4;
            high_nibble = false;
        }
        else
        {
            ok = true;
            return b;
        }
    }
}

bool parse_prog_command(ParseCmd &cmd)
{
    serial_rd_ptr = 0;

    uint8_t bcc = 0;

    bool ok = false;
    cmd.prog_addr = parse_hex_byte(ok, bcc);
    if (!ok)
        return false;

    int len = 0;
    cmd.command = parse_hex_byte(ok, bcc);
//         if (test_next_str("SET",     len, bcc)) cmd.command = OP_SET;
//    else if (test_next_str("MOV",     len, bcc)) cmd.command = OP_MOV;
//    else if (test_next_str("LDREG",   len, bcc)) cmd.command = OP_LDREG;
//    else if (test_next_str("IMAP",    len, bcc)) cmd.command = OP_IMAP;
//    else if (test_next_str("FMAP",    len, bcc)) cmd.command = OP_FMAP;
//    else if (test_next_str("RAND",    len, bcc)) cmd.command = OP_RAND;
//    else if (test_next_str("TPHASE",  len, bcc)) cmd.command = OP_TPHASE;
//    else if (test_next_str("SET_RGB", len, bcc)) cmd.command = OP_SET_RGB;
//    else if (test_next_str("SET_HSV", len, bcc)) cmd.command = OP_SET_HSV;
//    else if (test_next_str("HSV",     len, bcc)) cmd.command = OP_HSV;
//    else if (test_next_str("RGB",     len, bcc)) cmd.command = OP_RGB;
//    else if (test_next_str("PIX_HSV", len, bcc)) cmd.command = OP_PIX_HSV;
//    else if (test_next_str("PIX_RGB", len, bcc)) cmd.command = OP_PIX_RGB;
//    else if (test_next_str("PIX",     len, bcc)) cmd.command = OP_PIX;
//    else if (test_next_str("PIX_N",   len, bcc)) cmd.command = OP_PIX_N;
//    else if (test_next_str("ADD",     len, bcc)) cmd.command = OP_ADD;
//    else if (test_next_str("ADDF",    len, bcc)) cmd.command = OP_ADDF;
//    else if (test_next_str("MUL",     len, bcc)) cmd.command = OP_MUL;
//    else if (test_next_str("MULF",    len, bcc)) cmd.command = OP_MULF;
//    else if (test_next_str("DIV",     len, bcc)) cmd.command = OP_DIV;
//    else if (test_next_str("DIVF",    len, bcc)) cmd.command = OP_DIVF;
//    else if (test_next_str("SUB",     len, bcc)) cmd.command = OP_SUB;
//    else if (test_next_str("SUBF",    len, bcc)) cmd.command = OP_SUBF;
//    else if (test_next_str("FMOD",    len, bcc)) cmd.command = OP_FMOD;
//    else if (test_next_str("FMODF",   len, bcc)) cmd.command = OP_FMODF;
//    else if (test_next_str("SIN",     len, bcc)) cmd.command = OP_SIN;
//    else if (test_next_str("RET",     len, bcc)) cmd.command = OP_RET;
//    else
//        return false;

    cmd.arg1 = parse_hex_byte(ok, bcc);
    if (!ok)
        return false;
    cmd.arg2 = parse_hex_byte(ok, bcc);
    if (!ok)
        return false;
    cmd.arg3 = parse_hex_byte(ok, bcc);
    if (!ok)
        return false;

    if (serial_rd_ptr <= SERIAL_BUFFER_SIZE)
    {
        uint8_t cmd_bcc = SERIAL_BUFFER[serial_rd_ptr];
        if (cmd_bcc == bcc)
            return true;
        else
            return false;
    }
    else
        return false;
}

void read_serial_buffer(bool soft)
{
    while (true)
    {
        char c = ' ';
        if (soft) c = softSerial.read();
        else      c = Serial.read();

        if (c == -1)
            continue;

        if (serial_buf_ptr <= 0
                && (c == '\r' || c == '\n' || c == ' '))
            continue;

        if (c == '!')
        {
            if (soft) softSerial.println('S');
            else      Serial.println('S');

            continue;
        }

        if (!soft && c == '@')
        {
            char postFix = waitNextCharSerial();

            softSerial.begin(38400);

            sendAtCommand("AT", 0, false);
            sendAtCommand("AT+NAME?", 0, true);
            sendAtCommand("AT+NAME=WAVRVM", postFix, false);
            sendAtCommand("AT+NAME?", 0, true);

            softSerial.begin(9600);
            Serial.println("AT DONE");
            continue;
        }

        if (c == '#')
        {
            SERIAL_BUFFER[serial_buf_ptr] = '\0';
            ParseCmd cmd;
            if (parse_prog_command(cmd))
            {
                Serial.print("#:");
                Serial.print(cmd.prog_addr);
                Serial.print(", ");
                Serial.print(cmd.command);
                Serial.print(", ");
                Serial.print(cmd.arg1);
                Serial.print(", ");
                Serial.print(cmd.arg2);
                Serial.print(", ");
                Serial.println(cmd.arg3);

                if (soft) softSerial.println('@');
                else      Serial.println('@');
            }
            else
            {
                if (soft) softSerial.println('?');
                else      Serial.println('?');
            }
            serial_buf_ptr = 0;

        }

        if (c == '.')
        {
            if (soft) softSerial.println('E');
            else      Serial.println('E');

            return;
        }

        if (serial_buf_ptr >= SERIAL_BUFFER_SIZE)
            serial_buf_ptr = 0;

        SERIAL_BUFFER[serial_buf_ptr++] = c;
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
    softSerial.begin(9600);

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
    NEW_OP(OP_PIX_N,   0x00, 0x0F, 0x00);

    NEW_OP(OP_RAND,    0x06, 0x20, 0x3F);
    NEW_OP(OP_HSV,     0x06, 0x04, 0x05);
    NEW_OP(OP_PIX_N,   0x00, 0x0F, 0x00);

    NEW_OP(OP_RAND,    0x06, 0x30, 0x3F);
    NEW_OP(OP_HSV,     0x06, 0x04, 0x05);
    NEW_OP(OP_PIX_N,   0x00, 0x0F, 0x00);

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

    Serial.println("WAVR VM LED STARTED");
    Serial.flush();
    softSerial.println("WAVR VM LED STARTED");
    softSerial.flush();
}

void loop() {
    pixels.clear();
    unsigned long start_t = micros();
    exec_prog();
    pixels.show();

    unsigned long end_t = micros();

    if (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == '!')
        {
            Serial.println("S");

            read_serial_buffer(false);

            Serial.println("E");
        }
        else if (c == '.')
        {
            Serial.println("E");
        }
    }

    if (softSerial.available() > 0)
    {
        char c = softSerial.read();

        if (c == '!')
        {
            softSerial.println("S");

            read_serial_buffer(true);
        }
        else if (c == '.')
        {
            softSerial.println("E");
        }
    }

    if (counter % 120 == 0)
    {
        int fps = 1000000 / (end_t - start_t);
        softSerial.print("fps: ");
        softSerial.println(fps);
        Serial.print("fps: ");
        Serial.println(fps);
    }

    counter++;
}
