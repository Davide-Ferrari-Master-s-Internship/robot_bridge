#include "prbt_bridge/prbt_bridge.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "prbt_Bridge");

    prbt_bridge prbt;

    while (ros::ok()) {

        prbt.spinner();

    }

return 0;

}
