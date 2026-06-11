/*
SoLoud audio engine
Copyright (c) 2013-2020 Jari Komppa

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.
*/

#include "soloud.h"
#include "soloud_thread.h"

#if !defined(WITH_CTRU_NDSP) || !defined(__3DS__)

namespace SoLoud
{
	result ctru_ndsp_init(Soloud *aSoloud, unsigned int aFlags, unsigned int aSamplerate, unsigned int aBuffer, unsigned int aChannels)
	{
		return NOT_IMPLEMENTED;
	}
};

#else

#include <3ds.h>
#include <atomic>
#include <new>
#include <stdint.h>
#include <string.h>

namespace SoLoud
{
	static const int CTRU_NDSP_CHANNEL = 0;
	static const int CTRU_NDSP_WAVEBUFFERS = 3;
	struct CtruNdspData
	{
		CtruNdspData()
			: mSoloud(0)
			, mThreadHandle(0)
			, mSamples(0)
			, mRunning(false)
			, mSamplerate(0)
			, mSamplesPerBuffer(0)
			, mChannels(0)
		{
		}

		Soloud *mSoloud;
		Thread::ThreadHandle mThreadHandle;
		ndspWaveBuf mWaveBuf[CTRU_NDSP_WAVEBUFFERS];
		int16_t *mSamples;
		LightEvent mRefillEvent;
		std::atomic<bool> mRunning;
		unsigned int mSamplerate;
		unsigned int mSamplesPerBuffer;
		unsigned int mChannels;
	};

	static void ctru_ndsp_callback(void *aParam)
	{
		CtruNdspData *data = static_cast<CtruNdspData*>(aParam);
		if (data && data->mRunning.load(std::memory_order_acquire))
		{
			LightEvent_Signal(&data->mRefillEvent);
		}
	}

	static void ctru_ndsp_fill(CtruNdspData *aData, ndspWaveBuf *aWaveBuf)
	{
		aData->mSoloud->mixSigned16(aWaveBuf->data_pcm16, aData->mSamplesPerBuffer);
		DSP_FlushDataCache(
			aWaveBuf->data_pcm16,
			aData->mSamplesPerBuffer * aData->mChannels * sizeof(int16_t));
		ndspChnWaveBufAdd(CTRU_NDSP_CHANNEL, aWaveBuf);
	}

	static void ctru_ndsp_thread(void *aParam)
	{
		CtruNdspData *data = static_cast<CtruNdspData*>(aParam);

		while (data->mRunning.load(std::memory_order_acquire))
		{
			bool queued = false;
			for (int i = 0; i < CTRU_NDSP_WAVEBUFFERS; i++)
			{
				if (!data->mRunning.load(std::memory_order_acquire))
				{
					break;
				}
				if (data->mWaveBuf[i].status == NDSP_WBUF_FREE ||
					data->mWaveBuf[i].status == NDSP_WBUF_DONE)
				{
					ctru_ndsp_fill(data, &data->mWaveBuf[i]);
					queued = true;
				}
			}
			if (!queued && data->mRunning.load(std::memory_order_acquire))
			{
				LightEvent_Wait(&data->mRefillEvent);
			}
		}
	}

	static void ctru_ndsp_cleanup(Soloud *aSoloud)
	{
		if (0 == aSoloud->mBackendData)
		{
			return;
		}

		CtruNdspData *data = static_cast<CtruNdspData*>(aSoloud->mBackendData);
		ndspSetCallback(0, 0);
		data->mRunning.store(false, std::memory_order_release);
		LightEvent_Signal(&data->mRefillEvent);
		if (data->mThreadHandle)
		{
			Thread::wait(data->mThreadHandle);
			Thread::release(data->mThreadHandle);
		}

		ndspChnWaveBufClear(CTRU_NDSP_CHANNEL);
		ndspChnReset(CTRU_NDSP_CHANNEL);
		ndspExit();

		if (data->mSamples)
		{
			linearFree(data->mSamples);
		}
		delete data;
		aSoloud->mBackendData = 0;
	}

	result ctru_ndsp_init(Soloud *aSoloud, unsigned int aFlags, unsigned int aSamplerate, unsigned int aBuffer, unsigned int aChannels)
	{
		if (aSamplerate != 44100 || aChannels != 2 || aBuffer == 0)
		{
			return INVALID_PARAMETER;
		}

		Result ndspResult = ndspInit();
		if (R_FAILED(ndspResult))
		{
			return UNKNOWN_ERROR;
		}

		CtruNdspData *data = new (std::nothrow) CtruNdspData;
		if (!data)
		{
			ndspExit();
			return OUT_OF_MEMORY;
		}

		data->mSoloud = aSoloud;
		data->mRunning.store(true, std::memory_order_release);
		data->mSamplerate = aSamplerate;
		data->mSamplesPerBuffer = aBuffer;
		data->mChannels = aChannels;
		LightEvent_Init(&data->mRefillEvent, RESET_ONESHOT);

		size_t bufferSamples = data->mSamplesPerBuffer * data->mChannels * CTRU_NDSP_WAVEBUFFERS;
		data->mSamples = static_cast<int16_t*>(linearAlloc(bufferSamples * sizeof(int16_t)));
		if (!data->mSamples)
		{
			delete data;
			ndspExit();
			return OUT_OF_MEMORY;
		}

		memset(data->mWaveBuf, 0, sizeof(data->mWaveBuf));
		for (int i = 0; i < CTRU_NDSP_WAVEBUFFERS; i++)
		{
			data->mWaveBuf[i].data_pcm16 =
				data->mSamples + (i * data->mSamplesPerBuffer * data->mChannels);
			data->mWaveBuf[i].nsamples = data->mSamplesPerBuffer;
			data->mWaveBuf[i].looping = false;
			data->mWaveBuf[i].status = NDSP_WBUF_DONE;
		}

		ndspSetOutputMode(NDSP_OUTPUT_STEREO);
		ndspChnReset(CTRU_NDSP_CHANNEL);
		ndspChnSetInterp(CTRU_NDSP_CHANNEL, NDSP_INTERP_POLYPHASE);
		ndspChnSetRate(CTRU_NDSP_CHANNEL, (float)aSamplerate);
		ndspChnSetFormat(CTRU_NDSP_CHANNEL, NDSP_FORMAT_STEREO_PCM16);

		float mix[12];
		memset(mix, 0, sizeof(mix));
		mix[0] = 1.0f;
		mix[1] = 1.0f;
		ndspChnSetMix(CTRU_NDSP_CHANNEL, mix);

		aSoloud->mBackendData = data;
		aSoloud->mBackendCleanupFunc = ctru_ndsp_cleanup;
		aSoloud->mBackendString = "3DS NDSP";
		aSoloud->postinit_internal(aSamplerate, data->mSamplesPerBuffer * aChannels, aFlags, aChannels);

		ndspSetCallback(ctru_ndsp_callback, data);
		data->mThreadHandle = Thread::createThread(ctru_ndsp_thread, data);
		if (0 == data->mThreadHandle)
		{
			ctru_ndsp_cleanup(aSoloud);
			return UNKNOWN_ERROR;
		}

		return 0;
	}
};

#endif
