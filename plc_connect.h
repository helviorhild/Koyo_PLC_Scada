#ifndef PLC_CONNECT_H
#define PLC_CONNECT_H
#include <iostream>
//#include <fstream>
#include <mutex>
#include <thread>
#include <vector>
//#include <pair>
#include <iterator>
#include <unordered_set>
#include <unordered_map>
#include <string>
#define SERIAL_PORT_READ_BUF_SIZE 256
#define BAUD_RATE 19200
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <string.h>
#include <math.h>

class plc_connect
{
public:
    plc_connect(const std::string &puerto1);
    void write_data(char *data,size_t largo);
    void agregar_comentario(const std::string &comentario);
  //  void  write_data_file(const std::string& comentario, char * comando, long largo);
     void read_plc_add_dir(const unsigned short int addr,const unsigned short int cnt);
     void read_plc_del_dir(const unsigned short int addr);
     void dir_deco(char *pos);
      void escribir_plc();
     void deco_data(void);
     void receive(void);
     const char *see_data(unsigned short int);
      char lrc(char *data,unsigned short int largo);
     //void start_plc(void);
      const char *multiline_text(void);
      virtual ~plc_connect(void);

private:
        int serial_port;
    char read_buf_raw1[SERIAL_PORT_READ_BUF_SIZE];
    long bytes_transferred;
std::vector<std::pair<unsigned short int,unsigned short int>> dir2read;
std::vector<std::pair<unsigned short int,unsigned short int>>::iterator actual_it;
std::vector<long> removed_pos;
std::unordered_map<unsigned short int,unsigned short int> dir2valor;
char expected[8];
unsigned short int stage;
char data_out[6];
std::mutex mutex1;
bool runing;
std::string multiline_text_buf;
};

#endif // PLC_CONNECT_H
