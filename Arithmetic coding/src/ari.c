#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "ari.h"

enum 
{
    MAX_VALUE = 0xffffffffffffffffU,
    MAX_FREQ = 0xffffffffU,
    FIRST_QTR = MAX_FREQ / 4 + 1U,
    HALF = FIRST_QTR * 2U,
    THIRD_QTR = FIRST_QTR * 3U,
    BLOCK = 20000000,
};

static uint32_t output_mask = 0x80;
static uint32_t input_mask = 0x80;
static uint32_t bits_to_output = 0;
static uint32_t bits_to_input = 0;
static uint32_t freqs[258];
static uint32_t freqs2[258];
static int32_t bits_to_follow = 0;
static int32_t count_updates = 0;
static int32_t change_flag = 0;
static uint64_t char_count = 0;
static int32_t current_table = 0;
static int32_t A;
static int32_t B;

struct Freq
{
    uint32_t l;
    uint32_t h;
    uint32_t div;
};

/*struct comp
{
    uint64_t outputed;
    uint64_t inputed;
};
static struct comp compr = {0,0};
*/ 
void
initialize_table()
{
    for (int32_t i = 0; i < 258; ++i) {
        freqs[i] = i;
    }
}

uint64_t new_agrs = 1500;

void
drop_table(void)
{
    //printf("%d %d \n", A, B);
    for (int32_t i = 1; i < 258; ++i) {
        freqs[i] = (freqs[i]) / 2 ;
        if (freqs[i] == 0) {
            freqs[i] = 1;
        }
        if (freqs[i - 1] >= freqs[i]) {
            freqs[i] = freqs[i-1] + 1;
        }
    }
}

void
update(int32_t c)
{
    if (char_count % BLOCK == 1) {
        drop_table();
    }
    new_agrs = new_agrs * 1000759 / 1000000;
    for (int32_t i = c + 1; i < 258; ++i) {
        freqs[i] += new_agrs;
    }
    if (char_count % 10000 == 0) {
        new_agrs = 1224;
    }
    if (freqs[257] >= MAX_FREQ / 2048 - 1 ) {
        for (int32_t i = 1; i < 258; ++i) {
            freqs[i] = (freqs[i]) / 2;
            if (freqs[i] == 0) {
                freqs[i] = 1;
            }
            if (freqs[i - 1] >= freqs[i]) {
                freqs[i] = freqs[i-1] + 1;
            }
        }
    }
}

struct Freq
get_freq(int32_t c)
{
    struct Freq tmp = {0};
    tmp.l = freqs[c];
    tmp.h = freqs[c + 1];
    tmp.div = freqs[257];
    update(c);
    return tmp;
}

void
output_bit(int32_t bit, FILE *ofp)
{
    if (bit) {
        bits_to_output |= output_mask;
    }
    output_mask >>= 1;
    if (!output_mask) {
        putc(bits_to_output, ofp);
        output_mask = 0x80;
        bits_to_output = 0;
    }
}

//inline

int32_t 
input_bit(FILE *ifp)
{
    int32_t current_bit;
    if (input_mask == 0x80) {
        bits_to_input = getc(ifp);
    }
    current_bit = input_mask & bits_to_input;
    input_mask >>= 1;
    if (input_mask == 0) {
        input_mask = 0x80;
    }
    return current_bit != 0;
}

void
compression(FILE *ifp, FILE *ofp)
{
    initialize_table();
    int32_t byte;
    uint64_t l = 0, h = MAX_FREQ;
    while(1) {
        char_count++;
        byte = getc(ifp);
        if (byte == EOF) {
            byte = 256;
        }
        struct Freq fr;
        fr = get_freq(byte);
        uint64_t segment =  h - l + 1;
        h = l + fr.h * segment / fr.div - 1;
        l = l + fr.l * segment / fr.div;
        while(1) {
            if (h < HALF) {
                output_bit(0, ofp);
                for (; bits_to_follow > 0; bits_to_follow--) {
                    output_bit(!0, ofp);
                }
            }
            else if (l >= HALF) {
                output_bit(1, ofp);
                for (; bits_to_follow > 0; bits_to_follow--) {
                    output_bit(!1, ofp);
                }
                l -= HALF; h -= HALF;
            }
            else if ((l >= FIRST_QTR) && (h < THIRD_QTR)) {
                bits_to_follow++;
                l -= FIRST_QTR; h -= FIRST_QTR;
            } else 
                break;
            l += l; h += h + 1;
            l &= MAX_FREQ; h &= MAX_FREQ;
        }
        if (byte == 256) {
            break;
        }
    }
    bits_to_follow++;
    int c;
    c = (l < FIRST_QTR) ? 0 : 1;
    output_bit(c, ofp);
    for (; bits_to_follow > 0; bits_to_follow--) {
        output_bit(!c, ofp);
    }
    if (output_mask != 0) {
        putc(bits_to_output, ofp);
    }
}

struct Freq // added bsearch
get_char(uint32_t value, int32_t *c)
{
    struct Freq fr = {0, 0, 0};
    uint32_t low = 0, high = 257;
    while (low != high) {
        int32_t mid = (high + low) / 2;
        if (freqs[mid] <= value) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    *c = low - 1;
    fr.h = freqs[low];
    fr.l = freqs[low - 1];
    fr.div = freqs[257];
    update(*c);
    return fr;
    return fr;
}

void 
decompression(FILE *ifp, FILE *ofp)
{
    initialize_table();
    uint64_t l = 0, h = MAX_FREQ, value = 0;
    for (int i = 0; i < 32; i++) {
        value <<= 1;
        value |= input_bit(ifp);
    }
    while(1) {
        char_count++;
        int32_t c;
        uint64_t segment = (h - l) + 1;
        uint64_t freqv = ((value - l + 1) * freqs[257]- 1) / segment;
        struct Freq fr = get_char(freqv, &c);
        if (c == 256) {
            break;
        }
        putc(c, ofp);
        h = l +  (segment * fr.h) / fr.div - 1;
        l = l +  (segment * fr.l) / fr.div;
        while(1) {
            if (h < HALF);
            else if (l >= HALF) {
                l -= HALF; h -= HALF; value -= HALF;
            }
            else if ((l >= FIRST_QTR) && (h < THIRD_QTR)) {
                l -= FIRST_QTR; h -= FIRST_QTR; value -= FIRST_QTR;
            }
            else 
                break;
            l += l; h += h + 1;
            l &= MAX_FREQ; h &= MAX_FREQ;
            value += value + input_bit(ifp);
        }
    }
}

void
compress_ari(char *ifile, char *ofile)
{
    char buf1[8192];
    char buf2[8192];
    FILE *ifp = (FILE *)fopen(ifile, "rb");
    FILE *ofp = (FILE *)fopen(ofile, "wb");
    setvbuf(ifp, buf1, _IOFBF, 8192);
    setvbuf(ofp, buf2, _IOFBF, 8192);
    compression(ifp, ofp);
    fclose(ifp);
    fclose(ofp);
}

void decompress_ari(char *ifile, char *ofile) {
    FILE *ifp = (FILE *)fopen(ifile, "rb");
    FILE *ofp = (FILE *)fopen(ofile, "wb");
    char buf1[8192];
    char buf2[8192];
    setvbuf(ifp, buf1, _IOFBF, 8192);
    setvbuf(ofp, buf2, _IOFBF, 8192);
    decompression(ifp, ofp);
    fclose(ifp);
    fclose(ofp);
}
