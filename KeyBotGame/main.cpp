#include "run.h"
#include "map.h"

using namespace key_bot_game;

int main() {    
    Run run;
    while (!run.run()) {}

    return 0;
}
