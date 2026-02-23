#include <Arduino.h>
#include <SD.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "dbc_runtime.h"
#include "can_signals_370z.h"

namespace
{
static constexpr uint16_t MAX_DBC_MESSAGES = 128;
static constexpr uint16_t MAX_DBC_SIGNALS = 2048;
static constexpr size_t STRING_POOL_SIZE = 32768;

struct runtime_msg_t
{
    can_message_t msg;
    uint16_t first_sig;
    uint16_t sig_count;
};

static runtime_msg_t g_msgs[MAX_DBC_MESSAGES];
static can_signal_t g_sigs[MAX_DBC_SIGNALS];
static char g_strpool[STRING_POOL_SIZE];
static uint16_t g_msg_count = 0;
static uint16_t g_sig_count = 0;
static size_t g_pool_pos = 0;
static bool g_loaded = false;
static bool g_loading = false;
static char g_loaded_path[64] = {0};

static inline char *ltrim(char *s)
{
    while (*s && isspace((unsigned char)*s))
        s++;
    return s;
}

static inline void rtrim(char *s)
{
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1]))
        s[--n] = '\0';
}

static const char *pool_strdup(const char *s)
{
    size_t n = strlen(s) + 1;
    if (g_pool_pos + n > STRING_POOL_SIZE)
        return nullptr;
    char *dst = &g_strpool[g_pool_pos];
    memcpy(dst, s, n);
    g_pool_pos += n;
    return dst;
}

static bool parse_bo_line(char *line, uint32_t &id, char *name, uint8_t &dlc, char *src)
{
    // BO_ 853 SPEED: 7 ABS
    unsigned long id_ul = 0;
    unsigned int dlc_ui = 0;
    if (sscanf(line, "BO_ %lu %31[^:]: %u %31s", &id_ul, name, &dlc_ui, src) != 4)
        return false;
    id = (uint32_t)id_ul;
    dlc = (uint8_t)dlc_ui;
    return true;
}

static bool parse_sg_line(char *line, char *sig_name, uint16_t &start_bit, uint8_t &bit_len,
                          uint8_t &byte_order, uint8_t &is_signed, float &scale, float &offset, char *unit)
{
    // SG_ Speed1 : 7|16@0+ (0.00621,0) [0|406.972] "mph" Vector__XXX
    char *p = strstr(line, "SG_");
    if (!p)
        return false;
    p += 3;
    p = ltrim(p);

    // Signal name until space/colon.
    size_t i = 0;
    while (*p && !isspace((unsigned char)*p) && *p != ':' && i < 31)
        sig_name[i++] = *p++;
    sig_name[i] = '\0';
    if (sig_name[0] == '\0')
        return false;

    char *colon = strchr(p, ':');
    if (!colon)
        return false;
    p = ltrim(colon + 1);

    unsigned int sb = 0, bl = 0, bo = 0;
    char sign = '+';
    if (sscanf(p, "%u|%u@%u%c", &sb, &bl, &bo, &sign) != 4)
        return false;

    char *lp = strchr(p, '(');
    char *comma = lp ? strchr(lp + 1, ',') : nullptr;
    char *rp = comma ? strchr(comma + 1, ')') : nullptr;
    if (!lp || !comma || !rp)
        return false;

    char scale_buf[24] = {0};
    char offset_buf[24] = {0};
    size_t sl = (size_t)(comma - (lp + 1));
    size_t ol = (size_t)(rp - (comma + 1));
    if (sl >= sizeof(scale_buf) || ol >= sizeof(offset_buf))
        return false;
    memcpy(scale_buf, lp + 1, sl);
    memcpy(offset_buf, comma + 1, ol);
    scale = (float)atof(scale_buf);
    offset = (float)atof(offset_buf);

    unit[0] = '\0';
    char *q1 = strchr(rp, '"');
    if (q1)
    {
        char *q2 = strchr(q1 + 1, '"');
        if (q2 && q2 > q1 + 1)
        {
            size_t ul = (size_t)(q2 - (q1 + 1));
            if (ul > 15)
                ul = 15;
            memcpy(unit, q1 + 1, ul);
            unit[ul] = '\0';
        }
    }

    start_bit = (uint16_t)sb;
    bit_len = (uint8_t)bl;
    byte_order = (uint8_t)bo;
    is_signed = (sign == '-') ? 1 : 0;
    return true;
}

static int find_msg_index(uint32_t id)
{
    for (uint16_t i = 0; i < g_msg_count; i++)
    {
        if (g_msgs[i].msg.id == id)
            return (int)i;
    }
    return -1;
}

static void clear_runtime_db()
{
    g_msg_count = 0;
    g_sig_count = 0;
    g_pool_pos = 0;
    g_loaded = false;
    g_loaded_path[0] = '\0';
}

static bool has_dbc_extension(const char *name)
{
    if (!name)
        return false;
    size_t n = strlen(name);
    if (n < 4)
        return false;
    const char *ext = name + n - 4;
    return (tolower((unsigned char)ext[0]) == '.') &&
           (tolower((unsigned char)ext[1]) == 'd') &&
           (tolower((unsigned char)ext[2]) == 'b') &&
           (tolower((unsigned char)ext[3]) == 'c');
}
} // namespace

bool dbc_runtime_load_from_path(const char *path)
{
    if (!path || !path[0])
        return false;
    if (g_loading)
        return false;

    g_loading = true;
    clear_runtime_db();

    File f = SD.open(path, FILE_READ);
    if (!f)
    {
        g_loading = false;
        return false;
    }

    int current_msg_idx = -1;
    while (f.available())
    {
        String ls = f.readStringUntil('\n');
        ls.trim();
        if (ls.length() == 0)
            continue;

        char line[256];
        size_t n = ls.length();
        if (n >= sizeof(line))
            n = sizeof(line) - 1;
        memcpy(line, ls.c_str(), n);
        line[n] = '\0';

        if (strncmp(line, "BO_", 3) == 0)
        {
            if (g_msg_count >= MAX_DBC_MESSAGES)
                continue;

            uint32_t id = 0;
            char name[32] = {0};
            uint8_t dlc = 8;
            char src[32] = {0};
            if (!parse_bo_line(line, id, name, dlc, src))
                continue;

            runtime_msg_t &m = g_msgs[g_msg_count];
            m.msg.id = id;
            m.msg.name = pool_strdup(name);
            m.msg.source = pool_strdup(src);
            m.msg.dlc = dlc;
            m.first_sig = g_sig_count;
            m.sig_count = 0;
            m.msg.signals = &g_sigs[g_sig_count];
            m.msg.signal_count = 0;

            if (!m.msg.name || !m.msg.source)
                continue;

            current_msg_idx = g_msg_count;
            g_msg_count++;
            continue;
        }

        if (strncmp(line, "SG_", 3) == 0 && current_msg_idx >= 0)
        {
            if (g_sig_count >= MAX_DBC_SIGNALS)
                continue;

            char sig_name[32] = {0};
            uint16_t start_bit = 0;
            uint8_t bit_len = 0, byte_order = 0, is_signed = 0;
            float scale = 1.0f, offset = 0.0f;
            char unit[16] = {0};
            if (!parse_sg_line(line, sig_name, start_bit, bit_len, byte_order, is_signed, scale, offset, unit))
                continue;

            const char *name_ptr = pool_strdup(sig_name);
            const char *unit_ptr = pool_strdup(unit);
            if (!name_ptr || !unit_ptr)
                continue;

            can_signal_t &s = g_sigs[g_sig_count];
            memset(&s, 0, sizeof(s));
            s.name = name_ptr;
            s.start_byte = (uint8_t)(start_bit / 8);
            s.start_bit = (uint8_t)(start_bit % 8);
            s.bit_length = bit_len;
            s.type = SIG_DBC_GENERIC;
            s.scale = scale;
            s.offset = offset;
            s.unit = unit_ptr;
            s.dbc_start_bit = start_bit;
            s.dbc_byte_order = byte_order;
            s.dbc_is_signed = is_signed;

            g_sig_count++;
            g_msgs[current_msg_idx].sig_count++;
            g_msgs[current_msg_idx].msg.signal_count = (uint8_t)g_msgs[current_msg_idx].sig_count;
            continue;
        }
    }
    f.close();

    if (g_msg_count > 0 && g_sig_count > 0)
    {
        strncpy(g_loaded_path, path, sizeof(g_loaded_path) - 1);
        g_loaded_path[sizeof(g_loaded_path) - 1] = '\0';
        g_loaded = true;
    }

    g_loading = false;
    return g_loaded;
}

bool dbc_runtime_load_from_sd_root()
{
    File root = SD.open("/");
    if (!root)
        return false;

    bool loaded = false;
    File entry = root.openNextFile();
    while (entry)
    {
        if (!entry.isDirectory())
        {
            const char *name = entry.name();
            if (has_dbc_extension(name))
            {
                char path[96];
                if (name[0] == '/')
                    snprintf(path, sizeof(path), "%s", name);
                else
                    snprintf(path, sizeof(path), "/%s", name);
                entry.close();
                root.close();
                loaded = dbc_runtime_load_from_path(path);
                return loaded;
            }
        }
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    return false;
}

bool dbc_runtime_is_loaded()
{
    return g_loaded;
}

const char *dbc_runtime_loaded_path()
{
    return g_loaded ? g_loaded_path : nullptr;
}

const can_message_t *dbc_runtime_find_message(uint32_t id)
{
    if (!g_loaded || g_loading)
        return nullptr;
    int idx = find_msg_index(id);
    if (idx < 0)
        return nullptr;
    return &g_msgs[idx].msg;
}
