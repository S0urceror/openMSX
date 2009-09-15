// $Id$

// Zemina 90-in-1 cartridge
//
//  90 in 1 uses Port &H77 for mapping:
//    bits 0-5: selected 16KB page
//    bits 6-7: addressing mode...
//      00 = same page at 4000-7FFF and 8000-BFFF (normal mode)
//      01 = same page at 4000-7FFF and 8000-BFFF (normal mode)
//      10 = [page AND 3E] at 4000-7FFF, [page AND 3E OR 01] at 8000-BFFF
//           (32KB mode)
//      11 = same page at 4000-7FFF and 8000-BFFF, but 8000-BFFF has high 8KB
//           and low 8KB swapped (Namco mode)

#include "RomZemina90in1.hh"
#include "MSXCPUInterface.hh"
#include "MSXMotherBoard.hh"
#include "Rom.hh"
#include "serialize.hh"
#include "unreachable.hh"

namespace openmsx {

RomZemina90in1::RomZemina90in1(
		MSXMotherBoard& motherBoard, const XMLElement& config,
		std::auto_ptr<Rom> rom)
	: Rom8kBBlocks(motherBoard, config, rom)
{
	reset(EmuTime::dummy());
	getMotherBoard().getCPUInterface().register_IO_Out(0x77, this);
}

RomZemina90in1::~RomZemina90in1()
{
	getMotherBoard().getCPUInterface().unregister_IO_Out(0x77, this);
}

void RomZemina90in1::reset(EmuTime::param dummy)
{
	setUnmapped(0);
	setUnmapped(1);
	setUnmapped(6);
	setUnmapped(7);
	writeIO(0x77, 0, dummy);
}

void RomZemina90in1::writeIO(word /*port*/, byte value, EmuTime::param /*time*/)
{
	byte page = 2 * (value & 0x3F);
	switch (value & 0xC0) {
	case 0x00:
	case 0x40:
		setRom(2, page + 0);
		setRom(3, page + 1);
		setRom(4, page + 0);
		setRom(5, page + 1);
		break;
	case 0x80:
		setRom(2, (page & ~2) + 0);
		setRom(3, (page & ~2) + 1);
		setRom(4, (page |  2) + 0);
		setRom(5, (page |  2) + 1);
		break;
	case 0xC0:
		setRom(2, page + 0);
		setRom(3, page + 1);
		setRom(4, page + 1);
		setRom(5, page + 0);
		break;
	default:
		UNREACHABLE;
	}
}

byte* RomZemina90in1::getWriteCacheLine(word /*address*/) const
{
	return unmappedWrite;
}

REGISTER_MSXDEVICE(RomZemina90in1, "RomZemina90in1");

} // namespace openmsx
