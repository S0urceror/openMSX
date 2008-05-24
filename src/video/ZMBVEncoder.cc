// $Id$

// Code based on DOSBox-0.65

#include "ZMBVEncoder.hh"
#include <cassert>
#include <cstdlib>
#include <cstring>

namespace openmsx {

static const unsigned char DBZV_VERSION_HIGH = 0;
static const unsigned char DBZV_VERSION_LOW = 1;
static const unsigned char COMPRESSION_ZLIB = 1;
static const unsigned MAX_VECTOR = 16;
static const unsigned BLOCK_WIDTH  = MAX_VECTOR;
static const unsigned BLOCK_HEIGHT = MAX_VECTOR;
static const unsigned FLAG_KEYFRAME = 0x01;

const char* ZMBVEncoder::CODEC_4CC = "ZMBV";

ZMBVEncoder::ZMBVEncoder(unsigned width_, unsigned height_, unsigned bpp)
	: width(width_)
	, height(height_)
{
	setupBuffers(bpp);
	createVectorTable();
	memset(&zstream, 0, sizeof(zstream));
	deflateInit(&zstream, 6); // compression level

	// I did a small test: compression level vs compression speed
	//  (recorded Space Manbow intro, video only)
	//
	// level |  time  | size
	// ------+--------+----------
	//   0   | 1m12.6 | 139442594
	//   1   | 1m12.1 |   5217288
	//   2   | 1m10.8 |   4887258
	//   3   | 1m11.8 |   4610668
	//   4   | 1m13.1 |   3791932  <-- old default
	//   5   | 1m14.2 |   3602078
	//   6   | 1m14.5 |   3363766  <-- current default
	//   7   | 1m15.8 |   3333938
	//   8   | 1m25.0 |   3301168
	//   9   | 2m04.1 |   3253706
	//
	// Level 6 seems a good compromise between size/speed for THIS test.
}

ZMBVEncoder::~ZMBVEncoder()
{
	delete[] oldframe;
	delete[] newframe;
	delete[] work;
	delete[] output;
}

void ZMBVEncoder::createVectorTable()
{
	for (int s = 1; s <= 10; ++s) {
		for (int y = -s; y <= s; ++y) {
			for (int x = -s; x <= s; ++x) {
				if ((abs(x) == s) || (abs(y) == s)) {
					vectorTable.push_back(CodecVector(x, y));
				}
			}
		}
	}
}

void ZMBVEncoder::setupBuffers(unsigned bpp)
{
	switch (bpp) {
	case 15:
		format = ZMBV_FORMAT_15BPP;
		pixelsize = 2;
		break;
	case 16:
		format = ZMBV_FORMAT_16BPP;
		pixelsize = 2;
		break;
	case 32:
		format = ZMBV_FORMAT_32BPP;
		pixelsize = 4;
		break;
	default:
		assert(false);
	}

	pitch = width + 2 * MAX_VECTOR;
	unsigned bufsize = (height + 2 * MAX_VECTOR) * pitch * pixelsize + 2048;

	oldframe = new unsigned char[bufsize];
	newframe = new unsigned char[bufsize];
	memset(oldframe, 0, bufsize);
	memset(newframe, 0, bufsize);
	work = new unsigned char[bufsize];
	outputSize = neededSize();
	output = new unsigned char[outputSize];

	assert((width  % BLOCK_WIDTH ) == 0);
	assert((height % BLOCK_HEIGHT) == 0);
	unsigned xblocks = width / BLOCK_WIDTH;
	unsigned yblocks = height / BLOCK_HEIGHT;
	for (unsigned y = 0; y < yblocks; ++y) {
		for (unsigned x = 0; x < xblocks; ++x) {
			blockOffsets.push_back(
				((y * BLOCK_HEIGHT) + MAX_VECTOR) * pitch +
				(x * BLOCK_WIDTH) + MAX_VECTOR);
		}
	}
}

unsigned ZMBVEncoder::neededSize()
{
	unsigned f = pixelsize;
	f = f * width * height + 2 * (1 + (width / 8)) * (1 + (height / 8)) + 1024;
	return f + f / 1000;
}

template<class P>
unsigned ZMBVEncoder::possibleBlock(int vx, int vy, unsigned offset)
{
	int ret = 0;
	P* pold = &(reinterpret_cast<P*>(oldframe))[offset + (vy * pitch) + vx];
	P* pnew = &(reinterpret_cast<P*>(newframe))[offset];
	for (unsigned y = 0; y < BLOCK_HEIGHT; y += 4) {
		for (unsigned x = 0; x < BLOCK_WIDTH; x += 4) {
			if (pold[x] != pnew[x]) ++ret;
		}
		pold += pitch * 4;
		pnew += pitch * 4;
	}
	return ret;
}

template<class P>
unsigned ZMBVEncoder::compareBlock(int vx, int vy, unsigned offset)
{
	int ret = 0;
	P* pold = &(reinterpret_cast<P*>(oldframe))[offset + (vy * pitch) + vx];
	P* pnew = &(reinterpret_cast<P*>(newframe))[offset];
	for (unsigned y = 0; y < BLOCK_HEIGHT; ++y) {
		for (unsigned x = 0; x < BLOCK_WIDTH; ++x) {
			if (pold[x] != pnew[x]) ++ret;
		}
		pold += pitch;
		pnew += pitch;
	}
	return ret;
}

template<class P>
void ZMBVEncoder::addXorBlock(int vx, int vy, unsigned offset)
{
	P* pold = &(reinterpret_cast<P*>(oldframe))[offset + (vy * pitch) + vx];
	P* pnew = &(reinterpret_cast<P*>(newframe))[offset];
	for (unsigned y = 0; y < BLOCK_HEIGHT; ++y) {
		for (unsigned x = 0; x < BLOCK_WIDTH; ++x) {
			*reinterpret_cast<P*>(&work[workUsed]) = pnew[x] ^ pold[x];
			workUsed += sizeof(P);
		}
		pold += pitch;
		pnew += pitch;
	}
}

template<class P>
void ZMBVEncoder::addXorFrame()
{
	signed char* vectors = reinterpret_cast<signed char*>(&work[workUsed]);

	// Align the following xor data on 4 byte boundary
	unsigned blockcount = blockOffsets.size();
	workUsed = (workUsed + blockcount * 2 + 3) & ~3;
	for (unsigned b = 0; b < blockcount; ++b) {
		unsigned offset = blockOffsets[b];
		int bestvx = 0;
		int bestvy = 0;
		unsigned bestchange = compareBlock<P>(0, 0, offset);
		int possibles = 64;
		unsigned vectorCount = vectorTable.size();
		for (unsigned v = 0; v < vectorCount && possibles; ++v) {
			if (bestchange < 4) break;
			int vx = vectorTable[v].x;
			int vy = vectorTable[v].y;
			if (possibleBlock<P>(vx, vy, offset) < 4) {
				--possibles;
				unsigned testchange = compareBlock<P>(vx, vy, offset);
				if (testchange < bestchange) {
					bestchange = testchange;
					bestvx = vx;
					bestvy = vy;
				}
			}
		}
		vectors[b * 2 + 0] = (bestvx << 1);
		vectors[b * 2 + 1] = (bestvy << 1);
		if (bestchange) {
			vectors[b * 2 + 0] |= 1;
			addXorBlock<P>(bestvx, bestvy, offset);
		}
	}
}

void ZMBVEncoder::compressFrame(bool keyFrame, const void** lineData,
                               void*& buffer, unsigned& written)
{
	std::swap(newframe, oldframe); // replace oldframe with newframe

	// Reset the work buffer
	workUsed = 0;
	unsigned writeDone = 1;
	unsigned char* writeBuf = output;

	output[0] = 0; // first byte contains info about this frame
	if (keyFrame) {
		output[0] |= FLAG_KEYFRAME;
		KeyframeHeader* header =
			reinterpret_cast<KeyframeHeader*>(writeBuf + writeDone);
		header->high_version = DBZV_VERSION_HIGH;
		header->low_version = DBZV_VERSION_LOW;
		header->compression = COMPRESSION_ZLIB;
		header->format = format;
		header->blockwidth = BLOCK_WIDTH;
		header->blockheight = BLOCK_HEIGHT;
		writeDone += sizeof(KeyframeHeader);
		deflateReset(&zstream); // restart deflate
	}

	// copy lines (to add black border)
	unsigned linePitch = pitch * pixelsize;
	unsigned lineWidth = width * pixelsize;
	unsigned char* dest = newframe +
	                      pixelsize * (MAX_VECTOR + MAX_VECTOR * pitch);
	for (unsigned i = 0; i < height; ++i) {
		memcpy(dest, lineData[i], lineWidth);
		dest += linePitch;
	}

	// compress
	if (keyFrame) {
		// Add the full frame data
		unsigned char* readFrame =
			newframe + pixelsize * (MAX_VECTOR + MAX_VECTOR * pitch);
		for (unsigned i = 0; i < height; ++i) {
			memcpy(&work[workUsed], readFrame, lineWidth);
			readFrame += linePitch;
			workUsed += lineWidth;
		}
	} else {
		// Add the delta frame data
		switch (pixelsize) {
		case 2:
			addXorFrame<short>();
			break;
		case 4:
			addXorFrame<unsigned>();
			break;
		}
	}
	// Create the actual frame with compression
	zstream.next_in = static_cast<Bytef*>(work);
	zstream.avail_in = workUsed;
	zstream.total_in = 0;

	zstream.next_out = static_cast<Bytef*>(writeBuf + writeDone);
	zstream.avail_out = outputSize - writeDone;
	zstream.total_out = 0;
	deflate(&zstream, Z_SYNC_FLUSH);

	buffer = output;
	written = writeDone + zstream.total_out;
}

} // namespace openmsx
