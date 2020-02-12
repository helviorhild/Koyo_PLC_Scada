#include "plc_connect.h"

plc_connect::plc_connect(const std::string &puerto1)
        {

            //printf("Inicializando el puerto \n");
                serial_port = open(puerto1.c_str(), O_RDWR);
           // Check for errors
           if (serial_port < 0) {
               printf("__LINE__:Error %i from open: %s\n", errno, strerror(errno));
           }
           //configurar el puerto
           struct termios tty;
           memset(&tty, 0, sizeof tty);

           // Read in existing settings, and handle any error
           if(tcgetattr(serial_port, &tty) != 0) {
               printf("__LINE__:Error %i from tcgetattr: %s\n", errno, strerror(errno));
           }

           //tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
           tty.c_cflag |= PARENB;  // Set parity bit, enabling parity
           tty.c_cflag |=PARODD; //odd parity
           tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
           //tty.c_cflag |= CSTOPB;  // Set stop field, two stop bits used in communication
           //tty.c_cflag |= CS5; // 5 bits per byte
           //tty.c_cflag |= CS6; // 6 bits per byte
           //tty.c_cflag |= CS7; // 7 bits per byte
           tty.c_cflag |= CS8; // 8 bits per byte (most common)
           tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
           //tty.c_cflag |= CRTSCTS;  // Enable RTS/CTS hardware flow control
           tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
           tty.c_lflag &= ~ICANON;
           tty.c_lflag &= ~ECHO; // Disable echo
           tty.c_lflag &= ~ECHOE; // Disable erasure
           tty.c_lflag &= ~ECHONL; // Disable new-line echo
           tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
           tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
           tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
           tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
           tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
           // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT IN LINUX)
           // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT IN LINUX)
           tty.c_cc[VTIME] = 6;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
           tty.c_cc[VMIN] = 1;
           cfsetispeed(&tty, B19200);
           cfsetospeed(&tty, B19200);
           // Save tty settings, also checking for error
           if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
               printf("__LINE__:Error %i from tcsetattr: %s\n", errno, strerror(errno));
           }

            actual_it=dir2read.begin();
            runing=true;

        }

  void plc_connect::receive(void){
     bytes_transferred=0;
     while( bytes_transferred==0)bytes_transferred= read(serial_port,&read_buf_raw1,SERIAL_PORT_READ_BUF_SIZE);
    if(bytes_transferred<(expected[6]+expected[7])||bytes_transferred<(expected[4]+expected[5])){
         std::cout<<__LINE__<<":error receive data length too short \n";
        stage=0;

    }else {
         bool good=true;
        if(expected[5]!=0){
            if(strncmp(read_buf_raw1+expected[4],expected,static_cast<unsigned long>(expected[5]))!=0)good=false;
         //  std::cout<<stage<<"5"<<good<<'\n';
            }
        if(expected[7]!=0){//
            if(strncmp(read_buf_raw1+expected[6],expected+expected[5],static_cast<unsigned long>(expected[7]))!=0)good=false;
         // std::cout<<stage<<"7"<<good<<'\n';
        }
        if(good)++stage;
        else{

            std::cout<<__LINE__<<":error al decodificar stage=0  \n";
             for(int i=0;i<bytes_transferred;++i) std::cout<<':'<<(int)read_buf_raw1[i];
             std::cout<<'\n';
            stage=0;
}
}

}
void plc_connect::write_data(char *data,size_t largo){
  //for(int i=0;i<largo;++i)std::cout<<data[i];
 // std::cout<<std::endl;
if (write(serial_port,data, largo)!=largo)std::cout<<__LINE__<<":Error escritura puerto serie";

}


void plc_connect::agregar_comentario(const std::string &comentario)
{
    std::string enviar = "\n COMENT:";
        enviar += comentario;
//		write_data(enviar, nullptr, 0);

}/*
void  plc_connect::write_data_file(const std::string& comentario, char * comando, long largo) {
    boost::mutex::scoped_lock look(w_mutex);
    outfile << comentario;
    if (largo != 0)outfile.write(comando, largo);
}*/
void plc_connect::read_plc_add_dir(const unsigned short int addr,const unsigned short int cnt){//add some addr to read
  std::lock_guard look(mutex1);
    if(!removed_pos.empty()){
        dir2read[static_cast<unsigned long>(removed_pos.back())]=std::make_pair(addr,cnt);
        removed_pos.pop_back();
    }else {
            dir2read.push_back(std::make_pair(addr,cnt));
            actual_it=dir2read.begin();
      }

}
void plc_connect::read_plc_del_dir(const unsigned short int addr) //del some addr from read
{
    //for(std::vector::iterator it=dir2read.begin();it!=dir2read.end();++it)
    std::vector<std::pair<unsigned short int,unsigned short int>>::iterator it=dir2read.begin();
    while(it!=dir2read.end()&&it->first!=addr)++it;
    *it=std::make_pair(0,0);
    removed_pos.push_back(std::distance(dir2read.begin(),it));

}
void plc_connect::dir_deco(char *pos){
  std::lock_guard look(mutex1);
    int octalNumber=actual_it->first+1,rem,decimalNumber=0,i=0;
    while (octalNumber != 0)
    {
        rem = octalNumber % 10;
        octalNumber /= 10;
        decimalNumber += rem * pow(8, i);
        ++i;
    }
    if(decimalNumber<0xFFFF) sprintf(pos,"%04X",decimalNumber);
    else std::cout<<"decimal number to big \n";
}
void plc_connect::escribir_plc(){
  while(runing){
    while(dir2read.empty())sleep(2);
    if(stage>4){
        stage=0;
        if(++actual_it==dir2read.end())actual_it=dir2read.begin();
      }
 //   std::cout<<"Escribir PLC "<<stage<<" actual_it"<<actual_it->first<<std::endl;
  //     std::cout<<"Escribir PLC "<<stage<<std::endl;
    switch (stage) {
    case 0:
       { char data[]={0x4E,0x21,0x5};
        write_data(data,3);
        expected[0]=0x4E;
        expected[1]=0x21;
        expected[2]=0x6;
        expected[4]=0;//posicion Inicial
        expected[5]=3;//cantidad
        expected[6]=0;//posicion Inicial
        expected[7]=0;//cantidad
        receive();
    }
        break;
    case 1:
    {
        char data[17];
        data[0]=0x1;//SOH
        data[1]='0';//|
        data[2]='1';//\slave addresses
        data[3]='0';//read->0 write->8
        data[4]='1';//Data type Vmemory=1
        dir_deco(data+5);//5-8 memory address from octal to hexadecimal
        data[9]='0'; //|
        data[10]='0';//\Number of complete blocks
         sprintf(data+11,"%02d",2*actual_it->second);//11-12  Number of bytes (partial data blocks)
        data[13]='0';//|
        data[14]='0';//\Master  ID
        data[15]=0x17; //ETB
       data[16]=lrc(data+1,14); //16 checksum
       write_data(data,17);
       expected[0]=0x6;//ACK
       expected[1]=0;
       expected[2]=0;
       expected[4]=0;
       expected[5]=1; //Only want ACK
       expected[6]=0;
       expected[7]=0;
       receive();
    }
        break;
    case 2:
       {//Wait for data
        expected[0]=0x2;//STX start of text
        expected[1]=0x3;//ETX si se espera un solo paquete(hasta 256bytes) ETB(0x17) si paquete intermedio(más de un paquete)(no implementado)
        expected[2]=0;
        expected[4]=0;
        expected[5]=1;//espero 0x2 en posicion 0
        expected[6]=static_cast<char>((2*actual_it->second))+1;
        expected[7]=1;//espero un caracter en 2*Vmemory a leer
        receive();
    }
        break;
    case 3:
       {
          //deco data
        deco_data();
        char data[]={0x6};//return ACK
        write_data(data,1);
        expected[0]=0x4;//Expect some EOT
        expected[1]=0;
        expected[2]=0;
        expected[4]=0;
        expected[5]=1;
        expected[6]=0;
        expected[7]=0;
        receive();
    }
        break;
    case 4:
       {
        char data[]={0x4};//return EOT
        write_data(data,1);
        expected[0]=0;//Expect nothing
        expected[1]=0;
        expected[2]=0;
        expected[4]=0;
        expected[5]=0;
        expected[6]=0;
        expected[7]=0;
        stage=0;
        if(++actual_it==dir2read.end())actual_it=dir2read.begin();
      //  sleep(1);
        //receive();
    }
        break;
    default:
        stage=0;//Never get there
        break;
    }
}
}

void plc_connect::deco_data(void){
    if(read_buf_raw1[bytes_transferred-1]==lrc(read_buf_raw1+1,static_cast<unsigned short>(bytes_transferred-3)))
        {
    for(int i=0;i<bytes_transferred-3;i+=2){
               unsigned short int val=(read_buf_raw1[i+1]&0xF);
                    val+=((read_buf_raw1[i+1]>>4)&0xF)*10;
                    val+=(read_buf_raw1[i+2]&0xF)*100;
                    val+=((read_buf_raw1[i+2]>>4)&0xF)*1000;
                    dir2valor.insert_or_assign(actual_it->first+i/2,val);
                    //std::cout<<"\n val="<<val<<'\n';
           }}
            else {
               std::cout<<"\n__LINE__: error checksum"<<lrc(read_buf_raw1+1,static_cast<unsigned short>(bytes_transferred-3))<<" \n";
               for(int i=0;i<bytes_transferred;++i) std::cout<<(int)read_buf_raw1[i];
               std::cout<<'\n'<<read_buf_raw1[bytes_transferred]<<'\n';
           }

}
const char *plc_connect::see_data(unsigned short int addr){
 auto valid_data= dir2valor.find(addr);
 if(valid_data!=dir2valor.end()){
     std::cout<<valid_data->second<<'\n';
     snprintf(data_out,6,"%04d", valid_data->second);
   }
 else{
     std::cout<<addr<<':'<<dir2valor.size()<<':'<<dir2read.size()<<std::endl;
   }
 return data_out;
}
char plc_connect::lrc(char *data,unsigned short int largo)
{
    unsigned short int lrc=0;
    for(unsigned short int i=0;i<largo;++i){
                lrc=((lrc^data[i])&0xFF);
          }
    return static_cast<char>(lrc);
}
const char * plc_connect::multiline_text(){
  multiline_text_buf.clear();
for(std::unordered_map<unsigned short int,unsigned short int>::iterator it=dir2valor.begin();it!=dir2valor.end();++it)
{
    multiline_text_buf+=std::to_string(it->first);
    multiline_text_buf+='\t';
    multiline_text_buf+=std::to_string(it->second);
    multiline_text_buf+='\n';
}


  return multiline_text_buf.c_str();
}

 plc_connect::~plc_connect(void) {
   runing=false;
   close(serial_port);
   }


