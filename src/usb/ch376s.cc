#include "ch376s.hh"
#include "serialize.hh"
#include "unreachable.hh"
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <iomanip>

#define SPI_MISTER 0

namespace openmsx {

CH376s::CH376s(const DeviceConfig& config)
	: MSXDevice(config)
{

	// reset(getCurrentTime());
}

//#define CH_CMD_CHECK_EXIST 0x06
void CH376s::reset(EmuTime::param /*time*/)
{
    const char device[] = "/dev/tty.usbmodem123451";
    if (fp>0)
      close (fp);
    fp = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fp == -1) {
      printf( "failed to open port\n" );
      return;
    }
    if(!isatty(fp)) {
      printf( "not serial\n" );
      return;
    }
    fcntl(fp, F_SETFL, 0);
    
    struct termios  config;
    if(tcgetattr(fp, &config) < 0) {
      printf( "cannot get serial attributes\n" );
      return;
    }

    bzero(&config, sizeof(config));
    config.c_cflag |= CRTSCTS | CS8 | CLOCAL | CREAD;
    config.c_iflag |= IGNPAR;
    cfsetispeed (&config, B115200);
    cfsetospeed (&config, B115200);
    
    config.c_cc[VTIME]    = 5;   /* inter-character timer unused */
    config.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */
    
    tcflush(fp, TCIFLUSH);
    tcsetattr(fp, TCSANOW, &config);
    /*
    writeCommand (CH_CMD_CHECK_EXIST);
    writeData(190);
    uint8_t value;
    int bytesread = readData(&value);
    if (bytesread!=1 && value!='A')
    {
      printf ("CH376S not identified");
      return;
    }
    */
}

// LOW_LEVEL serial communication to CH376
///////////////////////////////////////////////////////////////////////////
std::string getCommandName(uint8_t command)
{
  std::string cmdstr;
  switch (command)
  {
    //case 0x04:  cmdstr = "CH_CMD_SET_SPEED";
    //            break;
    case 0x01:  cmdstr = "CH_CMD_GET_IC_VER";
                break;
    case 0x05:  cmdstr = "CH_CMD_RESET_ALL";
                break;
    case 0x06:  cmdstr = "CH_CMD_CHECK_EXIST";
                break;
    case 0x0b:  cmdstr = "CH_CMD_SET_REGISTER"; // SET_RETRY, SD0_INT
                break;
    case 0x0f:  cmdstr = "CH_CMD_DELAY_100US";
                break;
    case 0x13:  cmdstr = "CH_CMD_SET_USB_ADDR";
                break;
    case 0x15:  cmdstr = "CH_CMD_SET_USB_MODE";
                break;
    case 0x16:  cmdstr = "CH_CMD_TEST_CONNECT";
                break;
    //case 0x17:  cmdstr = "CH_CMD_ABORT_NAK";
    //            break;
    case 0x22:  cmdstr = "CH_CMD_GET_STATUS";
                break;
    case 0x27:  cmdstr = "CH_CMD_RD_USB_DATA0";
                break;
    case 0x2c:  cmdstr = "CH_CMD_WR_HOST_DATA";
                break;
    case 0x4e:  cmdstr = "CH_CMD_ISSUE_TKN_X";
                break;
    case 0x46:  cmdstr = "CH_CMD_GET_DESCR";
                break;
    case 0x41:  cmdstr = "CH_CMD_CLR_STALL";
                break;
    case 0x30:  cmdstr = "CH_CMD_DISK_CONNECT";
                break;
    case 0x31:  cmdstr = "CH_CMD_DISK_MOUNT";
                break;
    case 0x32:  cmdstr = "CH_CMD_FILE_OPEN";
                break;
    case 0x36:  cmdstr = "CH_CMD_FILE_CLOSE";
                break;                
    case 0x33:  cmdstr = "CH_CMD_FILE_ENUM_GO";
                break;                                
    case 0x2f:  cmdstr = "CH_CMD_SET_FILE_NAME";
                break;
    case 0x3a:  cmdstr = "CH_CMD_BYTE_READ";
                break;
    case 0x3b:  cmdstr = "CH_CMD_BYTE_RD_GO";
                break;        
    case 0x3c:  cmdstr = "CH_CMD_BYTE_WRITE";
                break;
    case 0x3d:  cmdstr = "CH_CMD_BYTE_WR_GO";
                break;         
    case 0x40:  cmdstr = "CH_CMD_DIR_CREATE";
                break;   
    case 0x34:  cmdstr = "CH_CMD_FILE_CREATE";
                break;    
    case 0x2d:  cmdstr = "CH_CMD_WR_REQ_DATA";
                break;      
    case 0x37:  cmdstr = "CH_CMD_DIR_INFO_READ";
                break;       
    case 0x39:  cmdstr = "CH_CMD_BYTE_LOCATE";
                break;            
    default:cmdstr = "UNKNOWN";
            break;
  }
  return cmdstr;
}

bool skip_read_data=false;
void CH376s::writeCommand (uint8_t command)
{
    if (fp<0)
      return;

#if SPI_MISTER 
    if (command==0) {
      std::clog << "endCommand ()" << std::endl;

      uint8_t cmd[] = {END_COMMAND};
      write (fp,cmd,sizeof(cmd));
    } 
    else 
    {
#endif
      if (command!=0x0f)
        std::clog << "writeCommand (" << getCommandName (command) << ") 0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (command) << std::endl;

      uint8_t cmd[] = {WR_COMMAND,command};
      write (fp,cmd,sizeof(cmd));
#if SPI_MISTER    
    }
#endif
    skip_read_data = false;
    current_command = command;
}
void CH376s::writeData (uint8_t data)
{
    if (fp<0)
      return;

    if (current_command==CH_CMD_WR_HOST_DATA)
    {
      if (output_buffer==nullptr) 
      {
        output_buffer_size = data+1;
        output_buffer = static_cast<uint8_t*> (malloc (output_buffer_size));
        output_buffer_ptr = output_buffer;
        output_buffer_cnt = output_buffer_size;
      }
      *output_buffer_ptr = data;
      output_buffer_ptr++;
      output_buffer_cnt--;
      if (output_buffer_cnt==0)
      {
        writeDataMultiple (output_buffer,output_buffer_size);
        free (output_buffer);
        output_buffer = nullptr;
        output_buffer_cnt = 0;
        current_command = 0;
      }
    }
    else 
    {
      if (current_command==CH_CMD_SET_FILE_NAME)
      {
        if (data!=0)
          std::clog << char (data);
        else
          std::clog << std::endl;
      }
      else 
      {
        std::clog << "writeData (0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (data) << ")" << std::endl;
      }
      uint8_t cmd[] = {WR_DATA,data};
      write (fp,cmd,sizeof(cmd));
    }
}

ssize_t CH376s::readData (uint8_t* new_value)
{
    if (fp<0)
      return 0;

    if (input_buffer!=nullptr && input_buffer_cnt>0)
    {
        *new_value = *input_buffer_ptr;
        input_buffer_ptr++;
        input_buffer_cnt--;

        if (current_command!=0x0f && !skip_read_data)
          std::clog << "0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (*new_value) << " = readData ()" << std::endl;

        return 1;
    }

    uint8_t cmd[] = {RD_DATA};
    write (fp,cmd,sizeof(cmd));
    size_t len = read (fp,new_value,1);

    if (current_command==CH_CMD_RD_USB_DATA0)
    {
      // free old and allocate new buffer
      if (input_buffer!=nullptr)
      {
        free (input_buffer);
        input_buffer=nullptr;
        input_buffer_ptr=nullptr;
        input_buffer_cnt=input_buffer_size=0;
      }

      input_buffer_size = *new_value;
      if (input_buffer_size>0)
      {
        input_buffer = static_cast<uint8_t*> (malloc (input_buffer_size));
        input_buffer_ptr = input_buffer;
        input_buffer_cnt = input_buffer_size;
        
        // read all data at once
        readDataMultiple(input_buffer, input_buffer_size);
      }
    }
    else 
    {
      if (current_command!=0x0f && !skip_read_data)
        std::clog << "0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (*new_value) << " = readData ()" << std::endl;
    }
      
    return len;
}
ssize_t CH376s::readDataMultiple (uint8_t* buffer,uint8_t len)
{
    if (fp<0)
      return 0;

    uint8_t* buffer_start = buffer;
    uint8_t cmd[] = {RD_DATA_MULTIPLE,len};
    write (fp,cmd,sizeof(cmd));

    uint8_t bytes_read = 0;
    while (bytes_read<len)
    {
        uint8_t bytes = read (fp,buffer,len);
        bytes_read += bytes;
        buffer += bytes;
    }
    assert (bytes_read==len);

    //uint8_t bytes = read (fp,buffer,len);
    //assert (bytes==len);

    skip_read_data = true;

    std::clog << "readDataMultiple[" << unsigned (bytes_read) << "]: ";
    for (int i=0;i<bytes_read;i++) {
      std::clog << (i==0?"":",") << "0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (buffer_start[i]);
    }
    std::clog << std::endl;

    return bytes_read;
}
ssize_t CH376s::writeDataMultiple (uint8_t* buffer,uint8_t len)
{
    if (fp<0)
      return 0;
    uint8_t cmd[] = {WR_DATA_MULTIPLE,len};
    write (fp,cmd,sizeof(cmd));
    write (fp,buffer,len);

    std::clog << "writeDataMultiple: ";
    for (int i=0;i<len;i++) {
      std::clog << (i==0?"":",") << "0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (buffer[i]);
    }
    std::clog << std::endl;

    return len;
}
ssize_t CH376s::readStatus (uint8_t* new_value)
{
    if (fp<0)
      return 0;
    uint8_t cmd[] = {RD_STATUS};
    write (fp,cmd,sizeof(cmd));
    
    ssize_t bytes = read (fp,new_value,1);
    //std::clog << "0x" << std::hex << std::setw(2) << std::setfill('0') << unsigned (*new_value) << " = readStatus ()" << std::endl;
    return bytes;
}

byte CH376s::readIO(word port, EmuTime::param /*time*/)
{
	byte result=0;
	switch (port & 0xff) {
    // MSXUSB
		case 0x10: // read/write data
			readData (&result);
			break;
		case 0x11: // read status/write command
			readStatus (&result);
			break;
    // ROOKIEDRIVE
		case 0x20: // read/write data
			readData (&result);
			break;
		case 0x21: // read status/write command
			readStatus (&result);
			break;
		case 0x22: // read/write data
			readData (&result);
			break;
		case 0x23: // read status/write command
			readStatus (&result);
			break;
		default: // unreachable, avoid warning
			printf ("readIO error port: %d",port& 0xff);
			UNREACHABLE; result = 255;
		}
	return result;
}

byte CH376s::peekIO(word port, EmuTime::param time) const
{
	byte result;
	result = (const_cast <CH376s*> (this))->readIO (port,time);
	return result;
}

void CH376s::writeIO(word port, byte value, EmuTime::param /*time*/)
{
	switch (port & 0xff) {
    // MSXUSB
		case 0x10: // read/write data
			writeData (value);
			break;
		case 0x11: // select register bank 1
			writeCommand (value);
			break;
    // ROOKIEDRIVE
    case 0x20: // read/write data
			writeData (value);
			break;
		case 0x21: // select register bank 1
			writeCommand (value);
			break;
    case 0x22: // read/write data
			writeData (value);
			break;
		case 0x23: // select register bank 1
			writeCommand (value);
			break;
		default:
			printf ("writeIO error port: %d",port);
			UNREACHABLE;
	}
}

template<typename Archive>
void CH376s::serialize(Archive& ar, unsigned /*version*/)
{
	ar.template serializeBase<MSXDevice>(*this);
}
INSTANTIATE_SERIALIZE_METHODS(CH376s);
REGISTER_MSXDEVICE(CH376s, "CH376s");

} // namespace openmsx
