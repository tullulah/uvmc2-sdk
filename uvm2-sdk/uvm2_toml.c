/* uvm2_toml.c — see uvm2_toml.h for the subset and why it is enough. */
#include <string.h>
#include "uvm2_toml.h"

typedef struct {
    char name[UVM2_TOML_NAME_MAX];
    int  count;                       /* how many [[name]] so far */
} array_seen;
#define ARRAYS_MAX 8

static int is_space(char c) { return c == ' ' || c == '\t' || c == '\r'; }

/* A quoted string starting at s[0] == '"': unescaped into out; returns what follows it. */
static const char *string_at(const char *s, const char *end, char *out)
{
    int n = 0;
    s++;
    while (s < end && *s != '"' && *s != '\n') {
        char c = *s++;
        if (c == '\\' && s < end) {
            c = *s++;
            if (c == 'n') c = '\n';
            else if (c == 't') c = '\t';
        }
        if (n < UVM2_TOML_VALUE_MAX - 1) out[n++] = c;
    }
    out[n] = 0;
    return (s < end && *s == '"') ? s + 1 : s;
}

int uvm2_toml_read(const char *text, uint32_t len, uvm2_toml_cb cb, void *user)
{
    const char *p = text, *end = text + len;
    char table[UVM2_TOML_NAME_MAX] = "";
    int index = 0;
    array_seen seen[ARRAYS_MAX];
    int nseen = 0;
    char key[UVM2_TOML_NAME_MAX], value[UVM2_TOML_VALUE_MAX];

    while (p < end) {
        const char *line_end = memchr(p, '\n', (size_t)(end - p));
        if (!line_end) line_end = end;
        while (p < line_end && is_space(*p)) p++;
        if (p < line_end && *p == '[') {
            const int array = (p + 1 < line_end && p[1] == '[');
            const char *n = p + (array ? 2 : 1);
            int k = 0;
            while (n < line_end && *n != ']' && k < UVM2_TOML_NAME_MAX - 1) {
                if (!is_space(*n)) table[k++] = *n;
                n++;
            }
            table[k] = 0;
            index = 0;
            if (array) {
                int i;
                for (i = 0; i < nseen && strcmp(seen[i].name, table) != 0; i++) { }
                if (i == nseen && nseen < ARRAYS_MAX) {
                    strcpy(seen[nseen].name, table);
                    seen[nseen].count = 0;
                    nseen++;
                }
                if (i < nseen) index = seen[i].count++;
            }
        } else if (p < line_end && *p != '#') {
            int k = 0;
            const char *q = p;
            if (*q == '"') { q = string_at(q, line_end, value); strncpy(key, value, sizeof key - 1); key[sizeof key - 1] = 0; }
            else {
                while (q < line_end && *q != '=' && !is_space(*q) && k < UVM2_TOML_NAME_MAX - 1) key[k++] = *q++;
                key[k] = 0;
            }
            while (q < line_end && (is_space(*q) || *q == '=')) q++;
            if (key[0] && q < line_end) {
                if (*q == '"') {
                    string_at(q, line_end, value);
                    if (!cb(user, table, index, key, value)) return 1;
                } else if (*q == '[') {
                    q++;
                    while (q < line_end && *q != ']') {
                        if (*q == '"') {
                            q = string_at(q, line_end, value);
                            if (!cb(user, table, index, key, value)) return 1;
                        } else q++;
                    }
                } else {
                    int v = 0;
                    while (q < line_end && !is_space(*q) && *q != '#' && v < UVM2_TOML_VALUE_MAX - 1) value[v++] = *q++;
                    value[v] = 0;
                    if (!cb(user, table, index, key, value)) return 1;
                }
            }
        }
        p = line_end + 1;
    }
    return 0;
}
