// $Id$

#ifndef __MSXMEGAROM_HH__
#define __MSXMEGAROM_HH__

#include "MSXRom.hh"
#include "MSXMemDevice.hh"
#include "EmuTime.hh"

class MSXMegaRom : public MSXMemDevice, public MSXRom
{
	public:
		/**
		 * Constructor
		 */
		MSXMegaRom(MSXConfig::Device *config);

		/**
		 * Destructor
		 */
		~MSXMegaRom();
		
		void init();
		
		byte readMem(word address, EmuTime &time);
		void writeMem(word address, byte value, EmuTime &time);

	protected:
		virtual MSXConfig::Device* GetDeviceConfig();

	private:
		int romSize;
		int mapperType;
		byte mapperMask;
		byte *internalMemoryBank[8]; // 4 blocks of 8kB starting at #4000
		bool enabledSCC;

		int retriefMapperType();
		//MSXSCC *cartridgeSCC; //TODO write an SCC :-)

};
#endif
