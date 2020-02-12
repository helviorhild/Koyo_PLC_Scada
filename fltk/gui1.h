// generated by Fast Light User Interface Designer (fluid) version 1.0304

#ifndef gui1_h
#define gui1_h
#include <FL/Fl.H>
#include "../plc_connect.h"
#include <math.h>
void callback(void*);
#include <FL/Fl_Double_Window.H>
extern Fl_Double_Window *flt_window;
#include <FL/Fl_Input.H>
extern Fl_Input *vmemory;
#include <FL/Fl_Button.H>
#include <FL/Fl_Spinner.H>
extern Fl_Spinner *Vm_count;
#include <FL/Fl_Output.H>
extern Fl_Output *mostrar_valor;
Fl_Double_Window* make_window(plc_connect *nplc);
#endif
