/*
 * FreeEEG32 driver for OpenViBE
 *
 * \author Jeremy Frey
 * \author Yann Renard
 *
 */
#ifndef __OpenViBE_AcquisitionServer_CDriverFreeEEG32_H__
#define __OpenViBE_AcquisitionServer_CDriverFreeEEG32_H__

#include "ovasIDriver.h"
#include "../ovasCHeader.h"

#include "../ovasCSettingsHelper.h"
#include "../ovasCSettingsHelperOperators.h"

#if defined TARGET_OS_Windows
 typedef void * FD_TYPE;
#elif defined TARGET_OS_Linux
 typedef OpenViBE::int32 FD_TYPE;
#else
#endif

#include <vector>
#include <deque>

namespace OpenViBEAcquisitionServer
{
	/**
	 * \class CDriverFreeEEG32
	 * \author Jeremy Frey
	 * \author Yann Renard
	 */
	class CDriverFreeEEG32 : public OpenViBEAcquisitionServer::IDriver
	{
	public:

		CDriverFreeEEG32(OpenViBEAcquisitionServer::IDriverContext& rDriverContext);
		virtual void release(void);
		virtual const char* getName(void);

		virtual OpenViBE::boolean initialize(const OpenViBE::uint32 ui32SampleCountPerSentBlock, OpenViBEAcquisitionServer::IDriverCallback& rCallback);
		virtual OpenViBE::boolean uninitialize(void);

		virtual OpenViBE::boolean start(void);
		virtual OpenViBE::boolean stop(void);
		virtual OpenViBE::boolean loop(void);

		virtual OpenViBE::boolean isConfigurable(void);
		virtual OpenViBE::boolean configure(void);
		virtual const OpenViBEAcquisitionServer::IHeader* getHeader(void) { return &m_oHeader; }

	public:

		typedef enum
		{
			ParserAutomaton_Default,
			ParserAutomaton_StartByteReceived,
			ParserAutomaton_SampleNumberReceived,
			ParserAutomaton_EEGSamplesReceived,
			ParserAutomaton_AccelerometerSamplesReceived,
		} EParserAutomaton;

	protected:

		OpenViBE::int32 interpret24bitAsInt32(const std::vector < OpenViBE::uint8 >& byteBuffer);
		OpenViBE::int32 interpret16bitAsInt32(const std::vector < OpenViBE::uint8 >& byteBuffer);
		OpenViBE::int16 parseByte(OpenViBE::uint8 ui8Actbyte);

		OpenViBE::boolean sendCommand(::FD_TYPE i32FileDescriptor, const char *sCommand, OpenViBE::boolean bWaitForResponse, OpenViBE::boolean bLogResponse, OpenViBE::uint32 ui32Timeout, std::string& sReply);
		OpenViBE::boolean resetBoard(::FD_TYPE i32FileDescriptor, bool bRegularInitialization);
		OpenViBE::boolean handleCurrentSample(OpenViBE::int32 packetNumber); // will take car of samples fetch from FreeEEG32 board, dropping/merging packets if necessary
		void updateDaisy(OpenViBE::boolean bQuietLogging); // update internal state regarding daisy module

	protected:

		OpenViBE::boolean openDevice(::FD_TYPE * pFileDescriptor, OpenViBE::uint32 ui32TtyNumber);
		void closeDevice(::FD_TYPE i32FileDescriptor);
		OpenViBE::uint32 writeToDevice(::FD_TYPE i32FileDescriptor, const void* pBuffer, const OpenViBE::uint32 ui32BufferSize);
		OpenViBE::uint32 readFromDevice(::FD_TYPE i32FileDescriptor, void* pBuffer, const OpenViBE::uint32 ui32BufferSize, const OpenViBE::uint64 ui64TimeOut=0);

	protected:

		SettingsHelper m_oSettings;

		OpenViBEAcquisitionServer::IDriverCallback* m_pCallback;
		OpenViBEAcquisitionServer::CHeader m_oHeader;

		::FD_TYPE  m_i32FileDescriptor;

		OpenViBE::CString m_sDriverName;
		OpenViBE::CString m_sTTYName;
		OpenViBE::CString m_sAdditionalCommands; // string to send possibly upon initialisation
		OpenViBE::uint32 m_ui32ChannelCount;
		OpenViBE::uint32 m_ui32DeviceIdentifier;
		OpenViBE::uint32 m_ui32ReadBoardReplyTimeout; // parameter com init string
		OpenViBE::uint32 m_ui32FlushBoardReplyTimeout;  // parameter com init string
		OpenViBE::boolean m_bDaisyModule; // daisy module attached or not

		// FreeEEG32 protocol related
		OpenViBE::int16 m_i16SampleNumber; // returned by the board
		OpenViBE::uint32 m_ui32ReadState; // position in the sample (see doc)
		OpenViBE::uint8 m_ui8SampleBufferPosition;// position in the buffer
		std::vector < OpenViBE::uint8 > m_vEEGValueBuffer; // buffer for one EEG value (int24)
		std::vector < OpenViBE::uint8 > m_vAccValueBuffer; // buffer for one accelerometer value (int16)
		const static OpenViBE::uint8 EEGValueBufferSize = 3; // int24 == 3 bytes
		const static OpenViBE::uint8 AccValueBufferSize = 2; // int16 == 2 bytes
//		const static OpenViBE::uint8 EEGValueCountPerSample = 8; // the board send EEG values 8 by 8 (will concatenate 2 samples with daisy module)
		const static OpenViBE::uint8 EEGValueCountPerSample = 32; // the board send EEG values 8 by 8 (will concatenate 2 samples with daisy module)
		const static OpenViBE::uint8 AccValueCountPerSample = 3; // 3 accelerometer data per sample

		OpenViBE::uint32 MissingSampleDelayBeforeReset; // in ms - value acquired from configuration manager
		OpenViBE::uint32 DroppedSampleCountBeforeReset; // in samples - value acquired from configuration manager
		OpenViBE::uint32 DroppedSampleSafetyDelayBeforeReset; // in ms - value acquired from configuration manager

		std::deque < OpenViBE::uint32 > m_vDroppedSampleTime;

		OpenViBE::float32 m_f32UnitsToMicroVolts; // convert from int32 to microvolt
		OpenViBE::float32 m_f32UnitsToRadians; // converts from int16 to radians
		OpenViBE::uint32 m_ui32ExtractPosition; // used to situate sample reading both with EEG and accelerometer data
		OpenViBE::uint32 m_ui32ValidAccelerometerCount;
		OpenViBE::int32  m_i32LastPacketNumber; // used to detect consecutive packets when daisy module is used

		// buffer for multibyte reading over serial connection
		std::vector < OpenViBE::uint8 > m_vReadBuffer;
		// buffer to store sample coming from FreeEEG32 -- filled by parseByte(), passed to handleCurrentSample()
		std::vector < OpenViBE::float32 > m_vSampleEEGBuffer;
		std::vector < OpenViBE::float32 > m_vSampleEEGBufferDaisy;
		std::vector < OpenViBE::float32 > m_vSampleAccBuffer;
		std::vector < OpenViBE::float32 > m_vSampleAccBufferTemp;

		// buffer to store aggregated samples
		std::vector < std::vector < OpenViBE::float32 > > m_vChannelBuffer; // buffer to store channels & chunks
		std::vector < OpenViBE::float32 > m_vSampleBuffer;

		bool m_bSeenPacketFooter; // extra precaution to sync packets

		// mechanism to call resetBoard() if no data are received
		OpenViBE::uint32 m_ui32Tick; // last tick for polling
		OpenViBE::uint32 m_ui32StartTime; // actual time since connection

		// sample storing for device callback and serial i/o
		std::vector < OpenViBE::float32 > m_vCallbackSample;

	private:

		typedef struct
		{
			OpenViBE::uint32 ui32DeviceVersion;
			OpenViBE::uint32 ui32DeviceChannelCount;

			OpenViBE::uint32 ui32BoardId;
			OpenViBE::uint32 ui32DaisyId;
			OpenViBE::uint32 ui32MocapId;

			char sBoardChipset[64];
			char sDaisyChipset[64];
			char sMocapChipset[64];
		} SDeviceInformation;

		SDeviceInformation m_oDeviceInformation;
	};
};

#endif // __OpenViBE_AcquisitionServer_CDriverFreeEEG32_H__
