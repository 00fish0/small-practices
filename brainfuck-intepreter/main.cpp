#include <bits/types/FILE.h>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <cassert> // Include the <cassert> header file

const int code_size = 30000, cell_size = 3000;
char code[code_size];
int cell[cell_size];
int code_ptr=0;
int cell_ptr=0;
int end;

int read(const char *path) {
    const char table[] = {'>', '<', '+', '-', '.', ',', '[', ']'};

    FILE * fp = freopen(path, "r", stdin);
    assert(fp);

    char ch;
    int read_ptr=0;
    while ((ch = getchar())!=EOF) {
        if(read_ptr>=code_size) assert(0);

        for(size_t i=0; i<sizeof(table)/sizeof(*table); i++) {
            if(table[i] == ch) {
                code[read_ptr++] = ch;
                break;
            }
        }
    }

    return read_ptr;
}

int jump(int pos) {
    static bool init = false;
    static int jmp_table[cell_size];

    if(!init) {
        int stack[3000];
        int len=0;
        for(int i=0;i<end; i++) {
            assert(len>=0);

            if(code[i] == '[') {
                stack[len++] = i;
            }
            else if(code[i] == ']') {
                jmp_table[i] = stack[len-1];
                jmp_table[stack[len-1]] = i;
                len--;
            }
        }
    }

    return jmp_table[pos];
}

void exec(char command, int pos) {
    switch (command) {
        case '<':
            cell_ptr--;
            break;

        case '>':
            cell_ptr++;
            break;

        case '+':
            cell[cell_ptr] ++;
            break;

        case '-':
            cell[cell_ptr] --;
            break;

        case '.':
            putchar(cell[cell_ptr]);
            break;

        case ',':
            cell[cell_ptr] = getchar();
            break;

        case '[':
            if(cell[cell_ptr] == 0)
                code_ptr = jump(pos);
            break;

        case ']':
            code_ptr = jump(pos) - 1;
            break;
        
        default:
            assert(0);
    }
}

int main(int argc, char **argv)
{
    // printf("Usage: bf <path/to/file>");
    assert(argc==2);

    memset(cell, 0, sizeof(cell));

    end = read(argv[1]);

    printf("%s\n", code);

    for(; code_ptr < end; code_ptr++) {
        exec(code[code_ptr], code_ptr);
    }
}