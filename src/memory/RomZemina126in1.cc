// $Id$

// Zemina 126-in-1 cartridge
//
// Information obtained by studying MESS sources:
//    0x4001 : 0x4000-0x7FFF
//    0x4002 : 0x8000-0xBFFF


#include "RomZemina126in1.hh"
#include "CacheLine.hh"
#include "Rom.hh"
#include "serialize.hh"

namespace openmsx {

RomZemina126in1::RomZemina126in1(
		MSXMotherBoard& motherBoard, const XMLElement& config,
		std::auto_ptr<Rom> rom)
	: Rom16kBBlocks(motherBoard, config, rom)
{
	reset(EmuTime::dummy());
}

void RomZemina126in1::reset(EmuTime::param /*time*/)
{
	setUnmapped(0);
	setRom(1, 0);
	setRom(2, 1);
	setUnmapped(3);
}

void RomZemina126in1::writeMem(word address, byte value, EmuTime::param /*time*/)
{
	if (address == 0x4000) {
		setRom(1, value);
	} else if (address == 0x4001) {
		setRom(2, value);
	}
}

byte* RomZemina126in1::getWriteCacheLine(word address) const
{
	if (address == (0x4000 & CacheLine::HIGH)) {
		return NULL;
	} else {
		return unmappedWrite;
	}
}

REGISTER_MSXDEVICE(RomZemina126in1, "RomZemina126in1");

} // namespace openmsx
