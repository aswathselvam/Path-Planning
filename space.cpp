#include "space.h"
#include <stdlib.h>    

void Space::init(){
    srand (1);
    start.x = 10;
    start.y = 10;

    goal.x = 10;
    goal.y = 10;
}