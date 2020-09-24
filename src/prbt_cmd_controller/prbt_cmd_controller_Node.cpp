#include "prbt_cmd_controller/prbt_cmd_controller.h"


int main(int argc, char **argv) {

    ros::init(argc, argv, "prbt_cmd_Controller");

    controller_functions cf;

    while (ros::ok()) {

        // std::cout << "\n\n" << "Manual Planning ? [y/n] ";
        std::string cm; cm = "y";
        // std::cin >> cm;

        if (cm == "y") {

            cf.prbt_Planning_Request();

            if (cf.abort) {break;}

            cf.prbt_Plan();
            cf.prbt_GOTO();
        

            std::cout << "\n\n" << "Continue Planning ? [y/n] ";
            std::string cp;
            std::cin >> cp;
            std::cout << "\n";

            if (cp == "n") {ros::shutdown();}  
        
        }

        else if (cm == "n") {
    
            cf.prbt_Test_Trajectory ();

        }

    }

return 0;

}
