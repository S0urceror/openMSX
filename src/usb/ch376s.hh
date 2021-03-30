#ifndef CH376S_HH
#define CH376S_HH

#include "MSXDevice.hh"

namespace openmsx {


class CH376s final : public MSXDevice
{
public:
	explicit CH376s(const DeviceConfig& config);

	void reset(EmuTime::param time) override;
	byte readIO(word port, EmuTime::param time) override;
	byte peekIO(word port, EmuTime::param time) const override;
	void writeIO(word port, byte value, EmuTime::param time) override;

	template<typename Archive>
	void serialize(Archive& ar, unsigned version);

private:
	// LOW_LEVEL serial communication to CH376
	///////////////////////////////////////////////////////////////////////////
	void writeCommand (uint8_t command);
	void writeData (uint8_t data);
	ssize_t readData (uint8_t* new_value);
	ssize_t readDataMultiple (uint8_t* buffer,uint8_t len);
	ssize_t writeDataMultiple (uint8_t* buffer,uint8_t len);
	ssize_t readStatus (uint8_t* new_value);
	ssize_t readInterrupt (uint8_t* new_value);

	int fp=0;
	uint8_t* input_buffer=nullptr;
	uint8_t* input_buffer_ptr=nullptr;
	int input_buffer_cnt=0;
	int input_buffer_size = 0;
	uint8_t current_command=0;
	uint8_t* output_buffer=nullptr;
	uint8_t* output_buffer_ptr=nullptr;
	int output_buffer_cnt=0;
	int output_buffer_size = 0;

	const uint8_t WR_COMMAND = 1;
	const uint8_t RD_STATUS = 2;
	const uint8_t WR_DATA = 3;
	const uint8_t RD_DATA = 4;
	const uint8_t RD_INT = 5;
	const uint8_t RD_DATA_MULTIPLE = 6;
	const uint8_t WR_DATA_MULTIPLE = 7;
	const uint8_t END_COMMAND = 8;

	const uint8_t  CH_CMD_RD_USB_DATA0 = 0x27;
	const uint8_t  CH_CMD_WR_HOST_DATA = 0x2c;
	const uint8_t  CH_CMD_SET_FILE_NAME = 0x2f;
};
SERIALIZE_CLASS_VERSION(CH376s, 1);

} // namespace openmsx

#endif
