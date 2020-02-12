#include "plc_connect.h"
#include "fluid/gui1.h"
int main(int argc, char **argv)
{
auto plc= new  plc_connect( "/dev/ttyS1");
auto window=make_window(plc);
std::thread t1{[&]{
               plc->escribir_plc();
               }};

 window->show(argc, argv);
 return Fl::run();
}
