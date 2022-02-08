#include <iostream>
#include <opencv2/core.hpp>
#include "space.h"


int main(){

    Space space;
    space.init();
    space.solve();

    // space.start
    // space.goal
    // space.obstacles
    // space.nodes
    
    return 0;
} 