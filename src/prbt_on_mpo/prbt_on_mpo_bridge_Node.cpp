#include "prbt_on_mpo/prbt_on_mpo_bridge.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "Robot_Bridge");

    bridge rb;

    while (ros::ok()) {

        rb.spinner();

    }

return 0;

}
