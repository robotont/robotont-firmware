/**
 * float_format.c
 *
 * Lightweight float-to-string conversion with ZERO dynamic memory allocation
 * 
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#define MAX_DECIMALS 6

//---------------------------------------------
// Powers of 10 (integer)
//---------------------------------------------
static const uint32_t pow10_table[MAX_DECIMALS + 1] =
{
    1u,
    10u,
    100u,
    1000u,
    10000u,
    100000u,
    1000000u
};

//---------------------------------------------
// Safe append helpers
//---------------------------------------------

static inline int append_char(char *buf, int idx, size_t max, char c)
{
    if ((size_t)idx < max - 1)
        buf[idx++] = c;
    return idx;
}

static inline int append_str(char *buf, int idx, size_t max, const char *s)
{
    while (*s && (size_t)idx < max - 1)
        buf[idx++] = *s++;
    return idx;
}

//---------------------------------------------
// Fast unsigned integer writer
//---------------------------------------------

static inline int write_u32(char *buf, int idx, size_t max, uint32_t v)
{
    char tmp[10]; // enough for uint32
    int len = 0;

    if (v == 0)
        return append_char(buf, idx, max, '0');

    while (v)
    {
        tmp[len++] = (char)('0' + (v % 10));
        v /= 10;
    }

    // reverse
    while (len--)
        idx = append_char(buf, idx, max, tmp[len]);

    return idx;
}

//---------------------------------------------
// Fast float writer (scaled integer)
//---------------------------------------------

static int write_float(char *buf,
                       int idx,
                       size_t max,
                       float value,
                       int decimals)
{
    if (decimals < 0) decimals = 0;
    if (decimals > MAX_DECIMALS) decimals = MAX_DECIMALS;

    // Handle NaN / Inf
    if (!isfinite(value))
        return append_str(buf, idx, max, "nan");

    bool negative = value < 0.0f;
    if (negative)
        value = -value;

    uint32_t mult = pow10_table[decimals];

    // Prevent overflow during scaling
    if (value > (float)UINT32_MAX / mult)
        return append_str(buf, idx, max, "inf");

    // SCALE FIRST (only float-heavy op)
    uint32_t scaled = (uint32_t)(value * mult + 0.5f);

    // eliminate negative zero
    if (scaled == 0)
        negative = false;

    if (negative)
        idx = append_char(buf, idx, max, '-');

    uint32_t int_part  = scaled / mult;
    uint32_t frac_part = scaled % mult;

    idx = write_u32(buf, idx, max, int_part);

    if (decimals > 0)
    {
        idx = append_char(buf, idx, max, '.');

        // leading zeros in fraction
        uint32_t div = mult / 10;
        while (div && frac_part < div)
        {
            idx = append_char(buf, idx, max, '0');
            div /= 10;
        }

        if (frac_part)
            idx = write_u32(buf, idx, max, frac_part);
        else
            idx = append_char(buf, idx, max, '0');
    }

    return idx;
}

//---------------------------------------------
// ODOM formatter
//---------------------------------------------

int format_odom(char *buffer,
                size_t buf_size,
                float px, float py, float pz,
                float vx, float vy, float vz)
{
    if (buf_size < 64) return 0;

    int idx = 0;

    idx = append_str(buffer, idx, buf_size, "ODOM:");

    idx = write_float(buffer, idx, buf_size, px, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, py, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, pz, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, vx, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, vy, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, vz, 3);

    idx = append_char(buffer, idx, buf_size, '\r');
    idx = append_char(buffer, idx, buf_size, '\n');

    buffer[idx] = '\0';
    return idx;
}

//---------------------------------------------
// Joint state formatter
//---------------------------------------------

int format_joint_states(char *buffer,
                        size_t buf_size,
                        float p0, float p1, float p2,
                        float v0, float v1, float v2,
                        float e0, float e1, float e2)
{
    if (buf_size < 96) return 0;

    int idx = 0;

    idx = append_str(buffer, idx, buf_size, "JS:");

    // positions (4 decimals)
    idx = write_float(buffer, idx, buf_size, p0, 4);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, p1, 4);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, p2, 4);
    idx = append_char(buffer, idx, buf_size, ':');

    // velocities (3 decimals)
    idx = write_float(buffer, idx, buf_size, v0, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, v1, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, v2, 3);
    idx = append_char(buffer, idx, buf_size, ':');

    // efforts (1 decimal)
    idx = write_float(buffer, idx, buf_size, e0, 1);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, e1, 1);
    idx = append_char(buffer, idx, buf_size, ':');

    idx = write_float(buffer, idx, buf_size, e2, 1);

    idx = append_char(buffer, idx, buf_size, '\r');
    idx = append_char(buffer, idx, buf_size, '\n');

    buffer[idx] = '\0';
    return idx;
}
