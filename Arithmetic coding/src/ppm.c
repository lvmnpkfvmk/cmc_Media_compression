#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "ppm.h"

enum
{
    MAX_VALUE = 0xffffffffffffffffU,
    MAX_FREQ = 0xffffffffU,
    FIRST_QTR = MAX_FREQ / 4 + 1U,
    HALF = FIRST_QTR * 2U,
    THIRD_QTR = FIRST_QTR * 3U,
    BLOCK = 8192,
};

static uint32_t output_mask = 0x80;
static uint32_t input_mask = 0x80;
static uint32_t bits_to_output = 0;
static uint32_t bits_to_input = 0;
static int32_t bits_to_follow = 0;
static uint8_t exclusions[256] = {0};
static uint64_t freqs[258] = {0};
static int32_t max_order = 6;
static int32_t order = 6;
static struct CM *line[12];
static struct CM **aline;

struct Freq
{
    uint32_t l;
    uint32_t h;
    uint32_t div;
};

struct array
{
    uint8_t symb;
    uint32_t count;
    struct CM *next;
};

struct CM
{
    int32_t symbols;
    uint32_t max_count;
    struct array *arr;
    struct CM *less;
};

void
delete_all(struct CM * tmp)
{
    if (tmp == NULL) {
        return;
    } else {
        for (int i = 0; i < tmp->symbols + 1; ++i) {
            delete_all(tmp->arr[i].next);
        }
        free(tmp->arr);
        free(tmp);
    }
}

void
Local_Order_Estimation()
{
    uint32_t max = 1; int32_t new_order = -1;
    for (int current_order = max_order; current_order > -1; current_order--) {
        struct CM *tmp = aline[current_order];
        if (tmp->max_count > max) {
            max = tmp->max_count;
            new_order = current_order;
        }
    }
    order = new_order;
}

void
initialize_table_ppm()
{
    aline = line + 1;
    aline[-1] = calloc(sizeof(struct CM), 1);
    aline[-1]->arr = calloc(sizeof(struct array), 256);
    for (int i = 0; i < 256; ++i) {
        aline[-1]->arr[i].count = 1;
        aline[-1]->arr[i].symb = i;
    }
    for (int i = 0; i < max_order + 1; ++i) {
        aline[i] = calloc(sizeof(struct CM), 1);
        aline[i]->arr = calloc(sizeof(struct array), 1);
        aline[i - 1]->symbols = 0;
        aline[i - 1]->max_count = 0;
        aline[i]->less = aline[i - 1];
        aline[i - 1]->arr[0].next = aline[i];
    }
    aline[-1]->symbols = 255;
    aline[-1]->max_count = 1;
    aline[max_order]->symbols = -1;
}

void
drop_table_ppm(struct CM* tmp)
{
    for (int32_t i = 0; i < tmp->symbols + 1; ++i) {
        tmp->arr[i].count /= 2;
        if (tmp->arr[i].count == 0) {
            tmp->arr[i].count = 1;
        }
        if (tmp->arr[i - 1].count <= tmp->arr[i].count) {
            tmp->arr[i - 1].count = tmp->arr[i].count + 1;
        }
    }
}

struct Freq
get_freq_ppm(int32_t byte, int *success)
{
    freqs[257] = 0;
    struct Freq fr;
    struct CM *tmp = aline[order];
    uint32_t maxcnt = 0;
    uint32_t q1 = 0;
    if (order == -2) {
        fr.div = 1;
        fr.l = 0;
        fr.h = 1;
        *success = 1;
        return fr;
    }
    freqs[255 - tmp->symbols] = 0;
    for (int i = 255 - tmp->symbols; i < 256; ++i) {
        int j = 255 - i;
        freqs[i + 1] = freqs[i];
        if (tmp->arr[j].count > 0) {
            if (exclusions[tmp->arr[j].symb] == 0) {
                freqs[i + 1] += tmp->arr[j].count;
            }
        }
        if (tmp->arr[j].count == 1) {
            ++q1;
        }
        if (tmp->arr[j].count > maxcnt)
            maxcnt = tmp->arr[j].count;
    }
    tmp->max_count = maxcnt;
    for (int i = 0; i < tmp->symbols; ++i) {
        if (tmp->arr[i].count > 0 ) {
            exclusions[tmp->arr[i].symb] = 1;
        }
    }
    if (0 < q1 && q1 < freqs[256] && 0) {
        freqs[257] = freqs[256];
    } else {
        freqs[257] = freqs[256] + tmp->symbols + 1; //ppmc
        if (freqs[257] == 0) {
            freqs[257] = 1;
        }
    }
    fr.div = freqs[257];
    for (int i = 0; i < tmp->symbols + 1; ++i) {
        if (tmp->arr[i].symb == byte) {
            if (tmp->arr[i].count != 0) {
                fr.l = freqs[255 - i];
                fr.h = freqs[256 - i];
                *success = 1;
                return fr;
            } else {
                break;
            }
        }
    }
    if (0 < q1 && q1 < freqs[256] && 0) {
        fr.h = q1;
        fr.l = 0;
    } else {
        fr.l = freqs[256];
        fr.h = freqs[257];
    }
    --order;
    *success = 0;
    return fr;
}

struct CM *
shift(struct CM * tmp, int32_t byte, int current_order)
{
    tmp = tmp->less;
    if (current_order == 0) {
        return tmp->arr[0].next;
    }
    for (int i = 0; i <= tmp->symbols; ++i) {
        if (tmp->arr[i].symb == byte) {
            if (tmp->arr[i].next != NULL) {
                return tmp->arr[i].next;
            } else {
                break;
            }
        }
    }
    struct CM *prev_context = shift(tmp, byte, current_order - 1);
    for (int i = 0; i < tmp->symbols + 1; ++i) {
        if (tmp->arr[i].symb == byte) {
            struct CM *next_context = calloc(sizeof(struct CM), 1);
            next_context->symbols = -1;
            tmp->arr[i].next = next_context;
            next_context->less = prev_context;
            return next_context;
        }
    }
    int k = ++tmp->symbols;
    tmp->arr = realloc(tmp->arr, sizeof(struct array) * (k + 1));
    tmp->arr[k].symb = byte;
    tmp->arr[k].count = 0;
    struct CM *next_context = calloc(sizeof(struct CM), 1);
    next_context->symbols = -1;
    tmp->arr[k].next = next_context;
    next_context->less = prev_context;
    return next_context;
}

void
tozero()
{
    for (int i = 0; i < 256; ++i) {
        exclusions[i] = 0;
    }
}

void
update_ppm(int32_t byte)
{
    order = order > -1 ? order : 0;
    for (int i = order; i <= max_order; ++i) {
        int32_t flag = 0;
        struct CM *tmp = aline[i];
        for (int j = 0; j < tmp->symbols + 1; ++j) {
            if (tmp->arr[j].symb == byte) {
                order = i;
                flag = 1;
            }
        }
        if (!flag) {
            break;
        }
    }
    int k;
    for (; order <= max_order; ++order) {
        int exist = 0;
        for (int i = 0; i < aline[order]->symbols + 1; ++i) {
            if (aline[order]->arr[i].symb == byte) {
                k = i;
                exist = 1;
                break;
            }
        }
        if (!exist) {
            k = ++aline[order]->symbols;
            aline[order]->arr =
                realloc(aline[order]->arr, sizeof(struct array) * (k + 1));
            aline[order]->arr[k].count = 0; aline[order]->arr[k].symb = byte;
            aline[order]->arr[k].next = NULL; aline[order]->max_count = 0;
        }
        aline[order]->arr[k].count += 1 ;
        if (aline[order]->arr[k].count >= MAX_FREQ / 4 - 1 ) {
            drop_table_ppm(aline[order]);
        }
    }
    --order;
    tozero();
    aline[max_order] = shift(aline[max_order], byte, max_order);
    for ( int i = max_order - 1; i > 0; --i) {
        aline[i] = aline[i + 1]->less;
    }
}

void
output_bit_ppm(int32_t bit, FILE *ofp)
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

int32_t
input_bit_ppm(FILE *ifp)
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
code(FILE *ifp, FILE *ofp, struct Freq fr, uint64_t *l, uint64_t *h)
{
    uint64_t segment = *h - *l + 1;
    *h = *l + fr.h * segment / fr.div - 1;
    *l = *l + fr.l * segment / fr.div;
    while (1) {
        if (*h < HALF) {
            output_bit_ppm(0, ofp);
            for (; bits_to_follow > 0; bits_to_follow--) {
                output_bit_ppm(!0, ofp);
            }
        } else if (*l >= HALF) {
            output_bit_ppm(1, ofp);
            for (; bits_to_follow > 0; bits_to_follow--) {
                output_bit_ppm(!1, ofp);
            }
        } else if ((*l >= FIRST_QTR) && (*h < THIRD_QTR)) {
            bits_to_follow++;
            *l -= FIRST_QTR;
            *h -= FIRST_QTR;
        } else
            break;
        *l += *l;
        *h += *h + 1;
        *l &= MAX_FREQ;
        *h &= MAX_FREQ;
    }
}

uint64_t cnt = 0;

void
compression_ppm(FILE *ifp, FILE *ofp)
{
    initialize_table_ppm();
    int32_t byte;
    uint64_t l = 0, h = MAX_FREQ;
    while(1) {
        byte = getc(ifp);
        ++cnt;
        int succes = 0;
        if (byte == EOF) {
            byte = -1;
        }
        if (cnt % BLOCK == 1) {
            Local_Order_Estimation();
        }
        do {
            struct Freq fr;
            fr = get_freq_ppm(byte, &succes);
            //printf("%d %d %d %c\n", fr.l, fr.h, fr.div, byte);
            code(ifp, ofp, fr, &l, &h);
        } while(!succes);
        if (byte == -1) {
            break;
        }
        update_ppm(byte);
    }
    bits_to_follow++;
    int c;
    c = (l < FIRST_QTR) ? 0 : 1;
    output_bit_ppm(c, ofp);
    for (; bits_to_follow > 0; bits_to_follow--) {
        output_bit_ppm(!c, ofp);
    }
    if (output_mask != 0) {
        putc(bits_to_output, ofp);
    }
}

uint32_t q1_d = 0;

struct Freq
get_char_ppm(uint64_t value, int32_t *c)
{
    struct Freq fr;
    if (order == -2) {
        *c = -1;
        fr.h = 1;
        fr.l = 0;
        fr.div = 1;
        return fr;
    }
    int count = 257;
    while (value < freqs[count]) {
        --count;
    }
    fr.h = freqs[count + 1];
    fr.l = freqs[count];
    fr.div = freqs[257];
    if (count == 256) {
        if (0 < q1_d && q1_d < freqs[256] && 0) {
            fr.h = q1_d;
            fr.l = 0;
        }
        --order;
        *c = 256;
        return fr;
    }
    *c = aline[order]->arr[255 - count].symb;
    return fr;
}

uint64_t
get_value(uint64_t value, FILE *ifp)
{
    for (int i = 0; i < 32; i++) {
        value <<= 1;
        value |= input_bit_ppm(ifp);
    }
    return value;
}

void
get_freqs()
{
    freqs[257] = 0;
    struct Freq fr;
    struct CM *tmp = aline[order];
    if (order == -2) {
        return;
    }
    uint32_t max = 0;
    q1_d = 0;
    freqs[255 - tmp->symbols] = 0;
    for (int i = 255 - tmp->symbols; i < 256; ++i) {
        int j = 255 - i;
        freqs[i + 1] = freqs[i];
        if (tmp->arr[j].count > 0) {
            if (exclusions[tmp->arr[j].symb] == 0) {
                freqs[i + 1] += tmp->arr[j].count;
                if (tmp->arr[j].count == 1) {
                    ++q1_d;
                }
            }
        }
        if (tmp->arr[j].count > max)
            max = tmp->arr[j].count;
    }
    tmp->max_count = max;
    if (0 < q1_d && q1_d < freqs[256] && 0) {
        freqs[257] = freqs[256];
    } else {
        freqs[257] = freqs[256] + tmp->symbols + 1;
        if (freqs[257] == 0) {
            freqs[257] = 1;
        }
    }
    for (int j = 0; j < tmp->symbols; ++j) {
        if (tmp->arr[j].count > 0) {
            exclusions[tmp->arr[j].symb] = 1;
        }
    }
}

void
decode(uint64_t *l, uint64_t *h, struct Freq fr,
      uint64_t segment, uint64_t *value, FILE *ifp) {
    *h = *l + (segment * fr.h) / fr.div - 1;
    *l = *l + (segment * fr.l) / fr.div;
    while (1) {
        if (*h < HALF);
        else if (*l >= HALF) {
            *l -= HALF;
            *h -= HALF;
            *value -= HALF;
        } else if ((*l >= FIRST_QTR) && (*h < THIRD_QTR)) {
            *l -= FIRST_QTR;
            *h -= FIRST_QTR;
            *value -= FIRST_QTR;
        } else
            break;
        *l += *l;
        *h += *h + 1;
        *l &= MAX_FREQ;
        *h &= MAX_FREQ;
        *value += *value + input_bit_ppm(ifp);
    }
}

void
decompression_ppm(FILE *ifp, FILE *ofp)
{
    initialize_table_ppm();
    uint64_t l = 0, h = MAX_FREQ, value = 0;
    value = get_value(value, ifp);
    int32_t c;
    while (1) {
        cnt++;
        if (cnt % BLOCK == 1) {
            Local_Order_Estimation();
        }
        do {
            get_freqs();
            uint64_t segment =  h - l + 1;
            uint64_t freqv = ((value - l + 1) * freqs[257] - 1) / segment;
            struct Freq fr = get_char_ppm(freqv, &c);
            //printf("%d %d %d %c\n", fr.l, fr.h, fr.div, c);
            decode(&l,&h,fr,segment, &value,ifp);
        } while (c == 256);
        if (c == -1)
            break;
        putc(c, ofp);
        update_ppm(c);
    }
}

void compress_ppm(char *ifile, char *ofile) {
    FILE *ifp = (FILE *)fopen(ifile, "rb");
    FILE *ofp = (FILE *)fopen(ofile, "wb");
    compression_ppm(ifp, ofp);
    delete_all(aline[0]);
    free(aline[-1]->arr);
    free(aline[-1]);
    fclose(ifp);
    fclose(ofp);
}

void decompress_ppm(char *ifile, char *ofile) {
    FILE *ifp = (FILE *)fopen(ifile, "rb");
    FILE *ofp = (FILE *)fopen(ofile, "wb");
    decompression_ppm(ifp, ofp);
    delete_all(aline[0]);
    free(aline[-1]->arr);
    free(aline[-1]);
    fclose(ifp);
    fclose(ofp);
}
