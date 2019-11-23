/*
 * FreeEEG32 driver for OpenViBE
 *
 * \author Jeremy Frey
 * \author Yann Renard
 *
 */

#include "ovasCDriverFreeEEG32.h"
#include "ovasCConfigurationFreeEEG32.h"

#include <toolkit/ovtk_all.h>

#include <system/ovCTime.h>
#include <system/ovCMemory.h>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <time.h>
#include <cstring>
#include <sstream>

#if defined TARGET_OS_Windows
 #include <windows.h>
 #include <winbase.h>
 #include <cstdio>
 #include <cstdlib>
 #include <commctrl.h>
 #include <winsock2.h> // htons and co.
 //#define TERM_SPEED 57600
// #define TERM_SPEED CBR_115200 // FreeEEG32 is a bit faster than others
 #define TERM_SPEED 921600 // FreeEEG32 is a bit faster than others
#elif defined TARGET_OS_Linux
 #include <cstdio>
 #include <unistd.h>
 #include <fcntl.h>
 #include <termios.h>
 #include <sys/select.h>
 #include <netinet/in.h> // htons and co.
 #include <unistd.h>
// #define TERM_SPEED B115200
// #define TERM_SPEED 1500000
 #define TERM_SPEED 921600
#else
#endif

#define boolean OpenViBE::boolean
using namespace OpenViBEAcquisitionServer;
using namespace OpenViBE;
using namespace OpenViBE::Kernel;

// packet number at initialization
#define UNINITIALIZED_PACKET_NUMBER -1
#define UNDEFINED_DEVICE_IDENTIFIER uint32(-1)
#define READ_ERROR uint32(-1)
#define WRITE_ERROR uint32(-1)

// start and stop bytes from FreeEEG32 protocl
#define SAMPLE_START_BYTE 0xA0
#define SAMPLE_STOP_BYTE 0xC0

// some constants related to the sendCommand
//#define AD7770_VREF 4.5  // reference voltage for ADC in AD7770.  set by its hardware
#define AD7770_VREF 2.5  // reference voltage for ADC in AD7770.  set by its hardware
//#define AD7770_GAIN 24.0  //assumed gain setting for AD7770.  set by its Arduino code
#define AD7770_GAIN 8.0  //assumed gain setting for AD7770.  set by its Arduino code

// configuration tokens
#define Token_MissingSampleDelayBeforeReset       "AcquisitionDriver_FreeEEG32_MissingSampleDelayBeforeReset"
#define Token_DroppedSampleCountBeforeReset       "AcquisitionDriver_FreeEEG32_DroppedSampleCountBeforeReset"
#define Token_DroppedSampleSafetyDelayBeforeReset "AcquisitionDriver_FreeEEG32_DroppedSampleSafetyDelayBeforeReset"

//___________________________________________________________________//
// Heavily inspired by OpenEEG code. Will override channel count and sampling late upon "daisy" selection. If daisy module is attached, will concatenate EEG values and average accelerometer values every two samples.
//                                                                   //

CDriverFreeEEG32::CDriverFreeEEG32(IDriverContext& rDriverContext)
	:IDriver(rDriverContext)
	,m_oSettings("AcquisitionServer_Driver_FreeEEG32", m_rDriverContext.getConfigurationManager())
	,m_pCallback(NULL)
	,m_ui32ChannelCount(EEGValueBufferSize+AccValueBufferSize)
	,m_ui32DeviceIdentifier(UNDEFINED_DEVICE_IDENTIFIER)
	,m_bDaisyModule(false)
{
	m_sAdditionalCommands="";
	m_ui32ReadBoardReplyTimeout=5000;
	m_ui32FlushBoardReplyTimeout=500;
	m_sDriverName = "FreeEEG32";

	m_oSettings.add("Header", &m_oHeader);
	m_oSettings.add("DeviceIdentifier", &m_ui32DeviceIdentifier);
	m_oSettings.add("ComInit", &m_sAdditionalCommands);
	m_oSettings.add("ReadBoardReplyTimeout", &m_ui32ReadBoardReplyTimeout);
	m_oSettings.add("FlushBoardReplyTimeout", &m_ui32FlushBoardReplyTimeout);
	m_oSettings.add("DaisyModule", &m_bDaisyModule);

	m_oSettings.load();

	MissingSampleDelayBeforeReset = (uint32)rDriverContext.getConfigurationManager().expandAsUInteger(Token_MissingSampleDelayBeforeReset, 1000);
	DroppedSampleCountBeforeReset = (uint32)rDriverContext.getConfigurationManager().expandAsUInteger(Token_DroppedSampleCountBeforeReset, 5);
	DroppedSampleSafetyDelayBeforeReset = (uint32)rDriverContext.getConfigurationManager().expandAsUInteger(Token_DroppedSampleSafetyDelayBeforeReset, 1000);

	// default parameter loaded, update channel count and frequency
	this->updateDaisy(true);
}

void CDriverFreeEEG32::release(void)
{
	delete this;
}

const char* CDriverFreeEEG32::getName(void)
{
	return m_sDriverName.toASCIIString();
}

//___________________________________________________________________//
//                                                                   //

void CDriverFreeEEG32::updateDaisy(bool bQuietLogging)
{
	// change channel and sampling rate according to daisy module
	CConfigurationFreeEEG32::SDaisyInformation l_oDaisyInformation=CConfigurationFreeEEG32::getDaisyInformation(m_bDaisyModule?CConfigurationFreeEEG32::DaisyStatus_Active:CConfigurationFreeEEG32::DaisyStatus_Inactive);

	m_oHeader.setSamplingFrequency(l_oDaisyInformation.iSamplingRate);
	m_oHeader.setChannelCount(l_oDaisyInformation.iEEGChannelCount+l_oDaisyInformation.iAccChannelCount);

	if(!bQuietLogging)
	{
		m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Status - " << CString(m_bDaisyModule?"Daisy":"** NO ** Daisy") << " module option enabled, " << m_oHeader.getChannelCount() << " channels -- " << (int)((m_bDaisyModule?2:1)*EEGValueCountPerSample) << " EEG and " << (int)AccValueCountPerSample << " accelerometer -- at " << m_oHeader.getSamplingFrequency() << "Hz.\n";
	}

	// microvolt for EEG channels
	for(int32 i=0; i<l_oDaisyInformation.iEEGChannelCount; i++)
	{
		m_oHeader.setChannelUnits(i, OVTK_UNIT_Volts, OVTK_FACTOR_Micro);
	}

	// undefined for accelerometer/extra channels
	for(int32 i=0; i<l_oDaisyInformation.iAccChannelCount; i++)
	{
		m_oHeader.setChannelUnits(l_oDaisyInformation.iEEGChannelCount+i, OVTK_UNIT_Unspecified, OVTK_FACTOR_Base);
	}
}

boolean CDriverFreeEEG32::initialize(
	const uint32 ui32SampleCountPerSentBlock,
	IDriverCallback& rCallback)
{
	if(m_rDriverContext.isConnected())
	{
		return false;
	}

	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Configured 'missing sample delay before reset' to " << MissingSampleDelayBeforeReset << " ; this can be changed in the openvibe configuration file setting the " << CString(Token_MissingSampleDelayBeforeReset) << " token\n";
	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Configured 'dropped sample count before reset' to " << DroppedSampleCountBeforeReset << " ; this can be changed in the openvibe configuration file setting the " << CString(Token_DroppedSampleSafetyDelayBeforeReset) << " token\n";
	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Configured 'dropped sample safety delay before reset' to " << DroppedSampleSafetyDelayBeforeReset << " ; this can be changed in the openvibe configuration file setting the " << CString(Token_DroppedSampleSafetyDelayBeforeReset) << " token\n";

	m_ui32ChannelCount=m_oHeader.getChannelCount();

	// Initializes buffer data structures
	m_vReadBuffer.clear();
	m_vReadBuffer.resize(1024*16); // 16 kbytes of read buffer
	m_vCallbackSample.clear();

	// change channel and sampling rate according to daisy module
	this->updateDaisy(false);

	// init state
	m_ui32ReadState=CDriverFreeEEG32::ParserAutomaton_Default;
	m_ui32ExtractPosition = 0;
	m_i16SampleNumber     = -1;
	m_bSeenPacketFooter   = true; // let's say we will start with header

	if(!this->openDevice(&m_i32FileDescriptor, m_ui32DeviceIdentifier))
	{
		return false;
	}

	// check board status and print response
//	if(!this->resetBoard(m_i32FileDescriptor, true))
	if(!this->resetBoard(m_i32FileDescriptor, false))
	{
		this->closeDevice(m_i32FileDescriptor);
		return false;
	}

	// prepare buffer for samples
	m_vSampleEEGBuffer.resize(EEGValueCountPerSample);
	m_vSampleEEGBufferDaisy.resize(EEGValueCountPerSample);
	m_vSampleAccBuffer.resize(AccValueCountPerSample);
	m_vSampleAccBufferTemp.resize(AccValueCountPerSample);

	// init buffer for 1 EEG value and 1 accel value
	m_vEEGValueBuffer.resize(EEGValueBufferSize);
	m_vAccValueBuffer.resize(AccValueBufferSize);
	m_vSampleBuffer.resize(m_ui32ChannelCount);

	m_pCallback=&rCallback;
	m_i32LastPacketNumber = UNINITIALIZED_PACKET_NUMBER;

	m_rDriverContext.getLogManager() << LogLevel_Debug << CString(this->getName()) << " driver initialized.\n";

	// init scale factor
	m_f32UnitsToMicroVolts = (float32) (float32) ((AD7770_VREF * 1000000) / ((pow(2.,23)-1) * AD7770_GAIN));
	m_f32UnitsToRadians = (float32) (0.002 / pow(2.,4)); // @aj told me - this is undocumented and may have been taken from the FreeEEG32 plugin for processing

#if 0
	uint32 l_f32UnitsToMicroVolts = (float32) (AD7770_VREF/(pow(2.,23)-1)/AD7770_GAIN*1000000.); // $$$$ The notation here is ambiguous
	::printf("CHECK THIS OUT : %g %g %g\n", l_f32UnitsToMicroVolts-m_f32UnitsToMicroVolts, l_f32UnitsToMicroVolts, m_f32UnitsToMicroVolts);
#endif

	return true;
}

boolean CDriverFreeEEG32::start(void)
{
	if(!m_rDriverContext.isConnected())
	{
		return false;
	}

	if(m_rDriverContext.isStarted())
	{
		return false;
	}

	m_rDriverContext.getLogManager() << LogLevel_Debug << CString(this->getName()) << " driver started.\n";
	return true;
}

boolean CDriverFreeEEG32::loop(void)
{
	if(!m_rDriverContext.isConnected())
	{
		return false;
	}

	// try to awake the board if there's something wrong
/*	if(System::Time::getTime() - m_ui32Tick > MissingSampleDelayBeforeReset)
	{
		m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << "No response for " << (uint32)MissingSampleDelayBeforeReset << "ms, will try recovery now (Note this may eventually be hopeless as the board may not reply to any command either).\n";
		if(!this->resetBoard(m_i32FileDescriptor, false))
		{
			m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed reseting board\n";
			return false;
		}
	}
*/
	// read datastream from device
	uint32 l_ui32ReadLength = this->readFromDevice(m_i32FileDescriptor, &m_vReadBuffer[0], m_vReadBuffer.size());
	if(l_ui32ReadLength == READ_ERROR)
	{
		m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Could not receive data from [" << m_sTTYName << "]\n";
		return false;
	}

	// pass bytes one by one to parser to extract samples
	for(uint32 i=0; i<l_ui32ReadLength; i++)
	{
		// will have effect only if complete sample/packet
		if(this->handleCurrentSample(this->parseByte(m_vReadBuffer[i])))
		{
			// One full packet was processed !
		}
	}

	// now deal with acquired samples
	if(m_vChannelBuffer.size()!=0)
	{
		if(m_rDriverContext.isStarted())
		{
			m_vCallbackSample.resize(m_ui32ChannelCount*m_vChannelBuffer.size());

			for(uint32 i=0, k=0; i<m_ui32ChannelCount; i++)
			{
				for(uint32 j=0; j<m_vChannelBuffer.size(); j++)
				{
					m_vCallbackSample[k++]=m_vChannelBuffer[j][i];
				}
			}
			m_pCallback->setSamples(&m_vCallbackSample[0], m_vChannelBuffer.size());
			m_rDriverContext.correctDriftSampleCount(m_rDriverContext.getSuggestedDriftCorrectionSampleCount());
		}
		m_vChannelBuffer.clear();
	}
	return true;
}

boolean CDriverFreeEEG32::stop(void)
{
	if(!m_rDriverContext.isConnected())
	{
		return false;
	}

	if(!m_rDriverContext.isStarted())
	{
		return false;
	}

	m_rDriverContext.getLogManager() << LogLevel_Debug << CString(this->getName()) << " driver stopped.\n";
	return true;
}

boolean CDriverFreeEEG32::uninitialize(void)
{
	if(!m_rDriverContext.isConnected())
	{
		return false;
	}

	if(m_rDriverContext.isStarted())
	{
		return false;
	}

	this->closeDevice(m_i32FileDescriptor);

	m_rDriverContext.getLogManager() << LogLevel_Debug << CString(this->getName()) << " driver closed.\n";

	// Uninitializes data structures
	m_vReadBuffer.clear();
	m_vCallbackSample.clear();
	m_sTTYName = "";

#if 0
	delete [] m_pSample;
	m_pSample=NULL;
#endif
	m_pCallback=NULL;

	return true;
}

//___________________________________________________________________//
//                                                                   //

boolean CDriverFreeEEG32::isConfigurable(void)
{
	return true;
}

boolean CDriverFreeEEG32::configure(void)
{
	CConfigurationFreeEEG32 m_oConfiguration(
		OpenViBE::Directories::getDataDir() + "/applications/acquisition-server/interface-FreeEEG32.ui",
		m_ui32DeviceIdentifier);

	m_oConfiguration.setAdditionalCommands(m_sAdditionalCommands);
	m_oConfiguration.setReadBoardReplyTimeout(m_ui32ReadBoardReplyTimeout);
	m_oConfiguration.setFlushBoardReplyTimeout(m_ui32FlushBoardReplyTimeout);
	m_oConfiguration.setDaisyModule(m_bDaisyModule);

	if(!m_oConfiguration.configure(m_oHeader))
	{
		return false;
	}

	m_sAdditionalCommands=m_oConfiguration.getAdditionalCommands();
	m_ui32ReadBoardReplyTimeout=m_oConfiguration.getReadBoardReplyTimeout();
	m_ui32FlushBoardReplyTimeout=m_oConfiguration.getFlushBoardReplyTimeout();
	m_bDaisyModule=m_oConfiguration.getDaisyModule();
	m_oSettings.save();

	this->updateDaisy(false);

	return true;
}

// Convert EEG value format from int24 MSB (network order) to int32 host
// TODO: check on big endian architecture
int32 CDriverFreeEEG32::interpret24bitAsInt32(const std::vector < uint8 >& byteBuffer)
{
	// create a big endian so that we could adapt to host architecture later on
	int32 newInt = (byteBuffer[2] << 24) | (byteBuffer[1] << 16) | byteBuffer[0] << 8;
	// depending on most significant byte, set positive or negative value
	if((newInt & 0x00008000) > 0)
	{
		newInt |= 0x000000FF;
	}
	else
	{
		newInt &= 0xFFFFFF00;
	}
	// convert back from big endian (network order) to host
	return htonl(newInt);
}

// Convert EEG value format from int16 MSB (network order) to int32 host
int32 CDriverFreeEEG32::interpret16bitAsInt32(const std::vector < uint8 >& byteBuffer)
{
	// create a big endian so that we could adapt to host architecture later on
	int32 newInt = (byteBuffer[1] << 24) | byteBuffer[0] << 16;
	// depending on most significant byte, set positive or negative value
	if((newInt & 0x00800000) > 0)
	{
		newInt |= 0x0000FFFF;
	}
	else
	{
		newInt &= 0xFFFF0000;
	}
	// convert back from big endian (network order) to host
	return htonl(newInt);
}

// return sample number once one is received (between 0 and 255, -1 if none)
// NB: will wait to get footer and then header in a row, may miss a packet but will prevent a bad sync with stream (thx BrainBay for the tip!)
OpenViBE::int16 CDriverFreeEEG32::parseByte(uint8 ui8Actbyte)
{
	// finished to read sample or not
	bool l_bSampleStatus = false;

	switch(m_ui32ReadState)
	{
		// Default state: wait for Start byte
		case CDriverFreeEEG32::ParserAutomaton_Default:
			// if first byte is not the one expected, won't go further
			if(ui8Actbyte==SAMPLE_STOP_BYTE)
			{
				m_bSeenPacketFooter = true;
			}
			else
			{
				if(ui8Actbyte==SAMPLE_START_BYTE && m_bSeenPacketFooter)
				{
					m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_StartByteReceived;
				}
				m_bSeenPacketFooter = false;
			}
			// reset sample info
			m_i16SampleNumber  = -1;
			m_ui32ExtractPosition = 0;
			m_ui8SampleBufferPosition = 0;
			break;

		// Start byte received, consider next byte as sample number
		case CDriverFreeEEG32::ParserAutomaton_StartByteReceived:
			m_i16SampleNumber = ui8Actbyte;
			m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_SampleNumberReceived;
			break;

		// Sample number received, next bytes hold the EEG data
		/*
		 * Note: values are 24-bit signed, MSB first
		 * Bytes 3-5: Data value for EEG channel 1
		 * Bytes 6-8: Data value for EEG channel 2
		 * Bytes 9-11: Data value for EEG channel 3
		 * Bytes 12-14: Data value for EEG channel 4
		 * Bytes 15-17: Data value for EEG channel 5
		 * Bytes 18-20: Data value for EEG channel 6
		 * Bytes 21-23: Data value for EEG channel 6
		 * Bytes 24-26: Data value for EEG channel 8
		 */
		case CDriverFreeEEG32::ParserAutomaton_SampleNumberReceived:
			if(m_ui32ExtractPosition < EEGValueCountPerSample)
			{
				// fill EEG buffer
				if(m_ui8SampleBufferPosition < EEGValueBufferSize)
				{
					m_vEEGValueBuffer[m_ui8SampleBufferPosition] = ui8Actbyte;
					m_ui8SampleBufferPosition++;
				}

				// we got EEG value
				if(m_ui8SampleBufferPosition == EEGValueBufferSize)
				{
					// fill EEG channel buffer, converting at the same time from 24 to 32 bits + scaling
					m_vSampleEEGBuffer[m_ui32ExtractPosition] = (float) this->interpret24bitAsInt32(m_vEEGValueBuffer)*m_f32UnitsToMicroVolts;

					// reset for next value
					m_ui8SampleBufferPosition = 0;
					m_ui32ExtractPosition++;

				}
			}

			// finished with EEG
			if(m_ui32ExtractPosition == EEGValueCountPerSample)
			{
				// next step: accelerometer
				m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_EEGSamplesReceived;
				// re-use the same variable to know position inside accelerometer block (I know, I'm bad!).
				m_ui32ExtractPosition=0;
				m_ui32ValidAccelerometerCount=0;
			}
			break;

		// EEG Samples received, next bytes hold the Accelerometer data
		/*
		 * Note: values are 16-bit signed, MSB first
		 * Bytes 27-28: Data value for accelerometer channel X
		 * Bytes 29-30: Data value for accelerometer channel Y
		 * Bytes 31-32: Data value for accelerometer channel Z
		 */
		case CDriverFreeEEG32::ParserAutomaton_EEGSamplesReceived:
			if(m_ui32ExtractPosition < AccValueCountPerSample)
			{
				// fill Acc buffer
				if(m_ui8SampleBufferPosition < AccValueBufferSize)
				{
					m_vAccValueBuffer[m_ui8SampleBufferPosition] = ui8Actbyte;
					m_ui8SampleBufferPosition++;
				}

				// we got Acc value
				if(m_ui8SampleBufferPosition == AccValueBufferSize)
				{
					// fill Acc channel buffer, converting at the same time from 16 to 32 bits
					m_vSampleAccBufferTemp[m_ui32ExtractPosition] = (float) this->interpret16bitAsInt32(m_vAccValueBuffer)*m_f32UnitsToRadians;
					if(m_vSampleAccBufferTemp[m_ui32ExtractPosition] != 0)
					{
						m_ui32ValidAccelerometerCount++;
					}

					// reset for next value
					m_ui8SampleBufferPosition = 0;
					m_ui32ExtractPosition++;
				 }
			}

			// finished with acc
			if(m_ui32ExtractPosition == AccValueCountPerSample)
			{
				// next step: footer
				m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_AccelerometerSamplesReceived;

				if(m_ui32ValidAccelerometerCount == AccValueCountPerSample)
				{
					m_vSampleAccBuffer.swap(m_vSampleAccBufferTemp);
				}
			}
			break;

		// Accelerometer Samples received, now expect a stop byte to terminate the frame
		case CDriverFreeEEG32::ParserAutomaton_AccelerometerSamplesReceived:
			// expected footer: perfect, returns sample number
			if(ui8Actbyte==SAMPLE_STOP_BYTE)
			{
				// we shall pass
				l_bSampleStatus = true;
				// we're ockay for next time
				m_bSeenPacketFooter = true;
			}
			// if last byte is not the one expected, discard whole sample
			else
			{
			}
			// whatever happened, it'll be the end of this journey
			m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_Default;
			break;

		// uh-oh, should not be there
		default:
			if(ui8Actbyte==SAMPLE_STOP_BYTE)
			{
				// we're ockay for next time
				m_bSeenPacketFooter = true;
			}
			m_ui32ReadState = CDriverFreeEEG32::ParserAutomaton_Default;
			break;
	}

	// if it's a GO, returns sample number, may trigger channel push
	if(l_bSampleStatus)
	{
		return m_i16SampleNumber;
	}

	// by default we're not ready
	return -1;
}

//___________________________________________________________________//
//                                                                   //

// if bWaitForResponse, will wait response before leaving the function (until timeout is reached)
// if bLogResponse, the actual response is sent to log manager
// ui32Timeout: time to sleep between each character written (in ms)
boolean CDriverFreeEEG32::sendCommand(::FD_TYPE i32FileDescriptor, const char* sCommand, bool bWaitForResponse, bool bLogResponse, uint32 ui32Timeout, std::string& sReply)
{
	uint32 l_ui32CommandSize = strlen(sCommand);
	sReply = "";

	// no command: don't go further
	if(l_ui32CommandSize == 0)
	{
		return true;
	}

	// write command to the board
	for(size_t i=0; i<l_ui32CommandSize; i++)
	{
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": Sending sequence to FreeEEG32 board [" << std::string(1, sCommand[i]).c_str() << "]\n";
		if(this->writeToDevice(i32FileDescriptor, &sCommand[i], 1) == WRITE_ERROR)
		{
			return false;
		}

		// wait for response
		if(bWaitForResponse)
		{
			// buffer for serial reading
			std::ostringstream l_oReadStream;

			uint64 l_ui64TimeOut = ui32Timeout;
			uint64 l_ui64StartTime=System::Time::getTime();
			boolean l_bFinished = false;
			while(System::Time::getTime() - l_ui64StartTime < l_ui64TimeOut && !l_bFinished)
			{
				uint32 l_ui32ReadLength = this->readFromDevice(i32FileDescriptor, &m_vReadBuffer[0], m_vReadBuffer.size(), 10);
				if(l_ui32ReadLength == READ_ERROR)
				{
					return false;
				}
				else
				{
					l_oReadStream.write(reinterpret_cast<const char*>(&m_vReadBuffer[0]), l_ui32ReadLength);

					// early stop when the "$$$" pattern is detected
					const std::string& l_sContent=l_oReadStream.str();
					const std::string l_sEarlyStop = "$$$";
					if(l_sContent.size() >= l_sEarlyStop.size())
					{
						if(l_sContent.substr(l_sContent.size()-l_sEarlyStop.size(), l_sEarlyStop.size()) == l_sEarlyStop)
						{
							l_bFinished = true;
						}
					}
				}
			}

			if(!l_bFinished)
			{
				m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": After " << l_ui64TimeOut << "ms, timed out while waiting for board response !\n";
			}

			// now log response to log manager
			if(bLogResponse)
			{
				// l_oReadStream stream to std::string and then to const to please log manager
				if(l_oReadStream.str().size() > 0)
				{
					m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": " << (l_bFinished?"Board response":"Partial board response") << " was (size=" << uint32(l_oReadStream.str().size()) << ") :\n";
					m_rDriverContext.getLogManager() << l_oReadStream.str().c_str() << "\n";
				}
				else
				{
					m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": Board did not reply !\n";
				}
			}

			// saves reply
			sReply += l_oReadStream.str();
		}
		else
		{
			// When no reply is expected, wait at least 100ms that the commands hits the device
			System::Time::sleep(100);
		}
	}

	return true;
}

boolean CDriverFreeEEG32::resetBoard(::FD_TYPE i32FileDescriptor, bool bRegularInitialization)
{
	uint32 l_ui32InitializeStartTime = System::Time::getTime();
	std::string l_sCommandReply;

	// stop/reset/default board
	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Stopping board streaming...\n";
	if(!this->sendCommand(i32FileDescriptor, "s", true, false, m_ui32FlushBoardReplyTimeout, l_sCommandReply)) // the waiting serves to flush pending samples after stopping the streaming
	{
		m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed in stopping board !\n";
		return false;
	}

	// regular initialization of the board (not for recovery)
	if(bRegularInitialization)
	{
		std::string l_sLine;

		// reset 32-bit board (no effect with 8bit board)
		m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Soft reseting of the board...\n";
		if(!this->sendCommand(i32FileDescriptor, "v", true, true, m_ui32ReadBoardReplyTimeout, l_sCommandReply))
		{
			m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed in soft reseting board !\n";
			return false;
		}

		// gets meaningful information from reply
		// WARNING - The parsing here is not very robust
		//           The parsing here is very dependant on the current protocol definition
		std::istringstream l_oCommandReplyInputStringStream(l_sCommandReply);
		::memset(&m_oDeviceInformation, 0, sizeof(m_oDeviceInformation));
		while(std::getline(l_oCommandReplyInputStringStream, l_sLine, '\n'))
		{
			if(l_sLine != "$$$")
			{
				::sscanf(l_sLine.c_str(), "FreeEEG32 V%u %u channel", &m_oDeviceInformation.ui32DeviceVersion, &m_oDeviceInformation.ui32DeviceChannelCount);
				::sscanf(l_sLine.c_str(), "On Board %s Device ID: 0x%x", m_oDeviceInformation.sBoardChipset, &m_oDeviceInformation.ui32BoardId);
				::sscanf(l_sLine.c_str(), "On Daisy %s Device ID: 0x%x", m_oDeviceInformation.sDaisyChipset, &m_oDeviceInformation.ui32DaisyId);
				::sscanf(l_sLine.c_str(), "%[^ ] Device ID: 0x%x", m_oDeviceInformation.sMocapChipset, &m_oDeviceInformation.ui32MocapId);
			}
		}

		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": Got board config\n";
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ":   Version: " << m_oDeviceInformation.ui32DeviceVersion << "\n";
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ":   Channel count: " << m_oDeviceInformation.ui32DeviceChannelCount << "\n";
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ":   Board (devid:" << m_oDeviceInformation.ui32BoardId << ", devchip:" << m_oDeviceInformation.sBoardChipset << ")\n";
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ":   Daisy (devid:" << m_oDeviceInformation.ui32DaisyId << ", devchip:" << m_oDeviceInformation.sDaisyChipset << ")\n";
		m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ":   Mocap (devid:" << m_oDeviceInformation.ui32MocapId << ", devchip:" << m_oDeviceInformation.sMocapChipset << ")\n";

		// verifies FreeEEG32 board presence
		if(m_oDeviceInformation.ui32DeviceVersion==0x00)
		{
			m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": The attached device does not look like an FreeEEG32 device\n";
			return false;
		}

		// verifies daisy module presence
		if(m_oDeviceInformation.ui32DaisyId==0x00 && m_bDaisyModule)
		{
			m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": The attached device does not match current configuration with Daisy module requested while not connected on the board\n";
			return false;
		}

		// After discussin with @Aj May 2016 it has been decided to disable
		// the daisy module when it was present and not requested instead of
		// checking its presence

#if 0
		// verifies daisy module absence
		if(m_oDeviceInformation.ui32DaisyId!=0x00 && !m_bDaisyModule)
		{
			m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": Configured without daisy module while present on the board, please unplug\n";
			return false;
		}
#else
		// Disables daisy module when necessary
		if(m_oDeviceInformation.ui32DaisyId!=0x00 && !m_bDaisyModule)
		{
			m_rDriverContext.getLogManager() << LogLevel_Trace << this->m_sDriverName << ": Daisy module present but not requested, will now be disabled\n";
			if(!this->sendCommand(i32FileDescriptor, "c", true, true, m_ui32ReadBoardReplyTimeout, l_sCommandReply))
			{
				m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed in disabling daisy module !\n";
				return false;
			}
		}
#endif

		// sends additional commands if necessary
		std::istringstream l_oAdditionalCommandsInputStringStream(m_sAdditionalCommands.toASCIIString());
		while(std::getline(l_oAdditionalCommandsInputStringStream, l_sLine, '\255'))
		{
			if(l_sLine.length() > 0)
			{
				m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Additional custom commands for initialization : [" << CString(l_sLine.c_str()) << "]\n";
				if(!this->sendCommand(i32FileDescriptor, l_sLine.c_str(), true, true, m_ui32ReadBoardReplyTimeout, l_sCommandReply))
				{
					m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed sending additional command [" << CString(l_sLine.c_str()) << "] !\n";
					return false;
				}
			}
		}
	}

	// start stream
	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Starting stream...\n";
	if(!this->sendCommand(i32FileDescriptor, "b", false, false, m_ui32ReadBoardReplyTimeout, l_sCommandReply))
	{
		m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed starting stream\n";
		return false;
	}

	// should start streaming!
	m_ui32StartTime = System::Time::getTime();
	m_ui32Tick = m_ui32StartTime;
	m_i32LastPacketNumber = UNINITIALIZED_PACKET_NUMBER;
	m_vDroppedSampleTime.clear();
	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Status ready (initialization took " << m_ui32Tick - l_ui32InitializeStartTime << "ms)\n";

	return true;
}

// update internal state (lastPacket, number of packet processed, etc.).
// returns true if a new sample is created
bool CDriverFreeEEG32::handleCurrentSample(int32 i32PacketNumber)
{
	bool l_bSampleOK = false; // true if a sample is added to m_vChannelBuffer
	// if == -1, current sample is incomplete or corrupted
	if(i32PacketNumber >= 0)
	{
		boolean l_bHasDroppedSamples = false;

		// check packet drop
		if( (m_i32LastPacketNumber != UNINITIALIZED_PACKET_NUMBER) && ((m_i32LastPacketNumber + 1) % 256 != i32PacketNumber) )
		{
			uint32 l_ui32DroppedSampleTime = System::Time::getTime() - m_ui32StartTime;
			while(m_vDroppedSampleTime.size() != 0 && l_ui32DroppedSampleTime - m_vDroppedSampleTime.front() > 1000)
			{
				m_vDroppedSampleTime.pop_front();
			}
			l_bHasDroppedSamples = true;

			m_rDriverContext.getLogManager() << LogLevel_Warning << this->m_sDriverName << ": Packet dropped! [Last,Current]=[" << (int) m_i32LastPacketNumber << "," << i32PacketNumber << "] (that was " << uint32(m_vDroppedSampleTime.size()+1) << " dropped samples lately)\n";

			if(l_ui32DroppedSampleTime > DroppedSampleSafetyDelayBeforeReset)
			{
				m_vDroppedSampleTime.push_back(l_ui32DroppedSampleTime);
				if(m_vDroppedSampleTime.size() > DroppedSampleCountBeforeReset)
				{
					m_rDriverContext.getLogManager() << LogLevel_Warning << this->m_sDriverName << ": Too many samples dropped, will now attempt to recover with a reset of the board\n";
					if(!this->resetBoard(m_i32FileDescriptor, false))
					{
						m_rDriverContext.getLogManager() << LogLevel_ImportantWarning << this->m_sDriverName << ": Did not succeed reseting board\n";
						return false;
					}
				}
			}
		}

		// no daisy module: push directly values
		if(!m_bDaisyModule)
		{
			// concatenate EEG and Acc
			// YRD NOTE: FreeEEG32 ACCELEROMETERS ARE SAMPLED WAY BELOW EEG (~25Hz, depends upon firmware)
			// YRD NOTE: CONSEQUENTLY, THIS CODE CREATES BLOCKS OF DUPLICATES ACCELEROMETERS SAMPLES

			std::copy(m_vSampleEEGBuffer.begin(), m_vSampleEEGBuffer.end(), m_vSampleBuffer.begin());
			std::copy(m_vSampleAccBuffer.begin(), m_vSampleAccBuffer.end(), m_vSampleBuffer.begin()+m_vSampleEEGBuffer.size());

			// copy them to current chunk
			m_vChannelBuffer.push_back(m_vSampleBuffer);
			l_bSampleOK = true;
		}
		// even: daisy, odd: first 8 channels
		else
		{
			// on odd packet, got complete sample
			if(i32PacketNumber % 2)
			{
				// won't concatenate if there was packet drop
				if(!l_bHasDroppedSamples)
				{
					// Concatenate standard and daisy EEG values and Acc
					// YRD NOTE: FreeEEG32 ACCELEROMETERS ARE SAMPLED WAY BELOW EEG (~25Hz, depends upon firmware)
					// YRD NOTE: CONSEQUENTLY, THIS CODE CREATES BLOCKS OF DUPLICATES ACCELEROMETERS SAMPLES

					std::copy(m_vSampleEEGBuffer.begin(),      m_vSampleEEGBuffer.end(),      m_vSampleBuffer.begin());
					std::copy(m_vSampleEEGBufferDaisy.begin(), m_vSampleEEGBufferDaisy.end(), m_vSampleBuffer.begin()+m_vSampleEEGBuffer.size());
					std::copy(m_vSampleAccBuffer.begin(),      m_vSampleAccBuffer.end(),      m_vSampleBuffer.begin()+m_vSampleEEGBuffer.size()+m_vSampleEEGBufferDaisy.size());

					// at last, add to chunk
					m_vChannelBuffer.push_back(m_vSampleBuffer);
					l_bSampleOK = true;
				}
			}
			// an even packet: it's Daisy, store values for later
			else
			{
				// swap may modify origin, but it's faster
				m_vSampleEEGBufferDaisy.swap(m_vSampleEEGBuffer);
			}
		}

		m_i32LastPacketNumber = i32PacketNumber;
	}

	// something to read: won't have to poll before "long"
	if(l_bSampleOK)
	{
		m_ui32Tick = System::Time::getTime();
	}

	return l_bSampleOK;
}

boolean CDriverFreeEEG32::openDevice(::FD_TYPE* pFileDescriptor, uint32 ui32TTYNumber)
{
	CString l_sTTYName;

	if(ui32TTYNumber == UNDEFINED_DEVICE_IDENTIFIER)
	{
		// Tries to find an existing port to connect to
		uint32 i=0;
		boolean l_bSuccess=false;
		do
		{
			l_sTTYName = CConfigurationFreeEEG32::getTTYFileName(i++);
			if(CConfigurationFreeEEG32::isTTYFile(l_sTTYName))
			{
				m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Automatically picked port [" << i << ":" << l_sTTYName << "]\n";
				l_bSuccess=true;
			}
		}
		while(!l_bSuccess && i<CConfigurationFreeEEG32::getMaximumTTYCount());

		if(!l_bSuccess)
		{
			m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": Port has not been configure and driver could not open any port\n";
			return false;
		}
	}
	else
	{
 		l_sTTYName = CConfigurationFreeEEG32::getTTYFileName(ui32TTYNumber);
	}

#if defined TARGET_OS_Windows

	DCB dcb = {0};
	*pFileDescriptor=::CreateFile(
		(LPCSTR)l_sTTYName.toASCIIString(),
		GENERIC_READ|GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	if(*pFileDescriptor == INVALID_HANDLE_VALUE)
	{
		m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": Could not open port [" << l_sTTYName << "]\n";
		return false;
	}

	if(!::GetCommState(*pFileDescriptor, &dcb))
	{
		m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": Could not get comm state on port [" << l_sTTYName << "]\n";
		return false;
	}

	// update DCB rate, byte size, parity, and stop bits size
	dcb.DCBlength = sizeof(dcb);
	dcb.BaudRate  = TERM_SPEED;
	dcb.ByteSize  = 8;
	dcb.Parity    = NOPARITY;
	dcb.StopBits  = ONESTOPBIT;
	dcb.EvtChar   = '\0';

	// update flow control settings
	dcb.fDtrControl       = DTR_CONTROL_ENABLE;
	dcb.fRtsControl       = RTS_CONTROL_ENABLE;
	dcb.fOutxCtsFlow      = FALSE;
	dcb.fOutxDsrFlow      = FALSE;
	dcb.fDsrSensitivity   = FALSE;;
	dcb.fOutX             = FALSE;
	dcb.fInX              = FALSE;
	dcb.fTXContinueOnXoff = FALSE;
	dcb.XonChar           = 0;
	dcb.XoffChar          = 0;
	dcb.XonLim            = 0;
	dcb.XoffLim           = 0;
	dcb.fParity           = FALSE;

	::SetCommState(*pFileDescriptor, &dcb);
	::SetupComm(*pFileDescriptor, 64/*1024*/, 64/*1024*/);
	::EscapeCommFunction(*pFileDescriptor, SETDTR);
	::SetCommMask (*pFileDescriptor, EV_RXCHAR | EV_CTS | EV_DSR | EV_RLSD | EV_RING);

#elif defined TARGET_OS_Linux

	struct termios l_oTerminalAttributes;

	if((*pFileDescriptor=::open(l_sTTYName.toASCIIString(), O_RDWR))==-1)
	{
		m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": Could not open port [" << l_sTTYName << "]\n";
		return false;
	}

	if(::tcgetattr(*pFileDescriptor, &l_oTerminalAttributes)!=0)
	{
		::close(*pFileDescriptor);
		*pFileDescriptor=-1;
		m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": terminal: tcgetattr() failed - did you use the right port [" << l_sTTYName << "] ?\n";
		return false;
	}

	/* l_oTerminalAttributes.c_cflag = TERM_SPEED | CS8 | CRTSCTS | CLOCAL | CREAD; */
	l_oTerminalAttributes.c_cflag = TERM_SPEED | CS8 | CLOCAL | CREAD;
	l_oTerminalAttributes.c_iflag = 0;
	l_oTerminalAttributes.c_oflag = OPOST | ONLCR;
	l_oTerminalAttributes.c_lflag = 0;
	if(::tcsetattr(*pFileDescriptor, TCSAFLUSH, &l_oTerminalAttributes)!=0)
	{
		::close(*pFileDescriptor);
		*pFileDescriptor=-1;
		m_rDriverContext.getLogManager() << LogLevel_Error << this->m_sDriverName << ": terminal: tcsetattr() failed - did you use the right port [" << l_sTTYName << "] ?\n";
		return false;
	}

#else

	return false;

#endif

	m_rDriverContext.getLogManager() << LogLevel_Info << this->m_sDriverName << ": Successfully opened port [" << l_sTTYName << "]\n";

	m_sTTYName = l_sTTYName;

	return true;
}

void CDriverFreeEEG32::closeDevice(::FD_TYPE i32FileDescriptor)
{
#if defined TARGET_OS_Windows
	::CloseHandle(i32FileDescriptor);
#elif defined TARGET_OS_Linux
	::close(i32FileDescriptor);
#else
#endif
}

uint32 CDriverFreeEEG32::writeToDevice(::FD_TYPE i32FileDescriptor, const void* pBuffer, const uint32 ui32BufferSize)
{
	int32 l_i32WrittenByteCount = 0;

#if defined TARGET_OS_Windows

	DWORD l_uiLength = 0;
	if(FALSE == ::WriteFile(i32FileDescriptor, pBuffer, ui32BufferSize, &l_uiLength, NULL))
	{
		return WRITE_ERROR;
	}
	l_i32WrittenByteCount = int32(l_uiLength);

#elif defined TARGET_OS_Linux

	l_i32WrittenByteCount = ::write(i32FileDescriptor, pBuffer, ui32BufferSize);
	if(l_i32WrittenByteCount < 0)
	{
		return WRITE_ERROR;
	}

#else

	return WRITE_ERROR;

#endif

	return uint32(l_i32WrittenByteCount);
}

uint32 CDriverFreeEEG32::readFromDevice(::FD_TYPE i32FileDescriptor, void* pBuffer, const uint32 ui32BufferSize, const uint64 ui64TimeOut)
{
#if defined TARGET_OS_Windows

	uint32 l_ui32ReadLength=0;
	uint32 l_ui32ReadOk=0;
	struct _COMSTAT l_oStatus;
	::DWORD l_dwState;

	if(::ClearCommError(i32FileDescriptor, &l_dwState, &l_oStatus))
	{
		l_ui32ReadLength=(l_oStatus.cbInQue<ui32BufferSize?l_oStatus.cbInQue:ui32BufferSize);
	}

	if(l_ui32ReadLength > 0)
	{
		if(FALSE == ::ReadFile(i32FileDescriptor, pBuffer, l_ui32ReadLength, (LPDWORD)&l_ui32ReadOk, 0))
		{
			return READ_ERROR;
		}
		return l_ui32ReadLength;
	}
	return 0;

#elif defined TARGET_OS_Linux

	fd_set  l_inputFileDescriptorSet;
	struct timeval l_timeout;
	bool l_bFinished=false;

	l_timeout.tv_sec=0;
	l_timeout.tv_usec=((ui64TimeOut>>20)*1000*1000)>>12;

	uint32 l_ui32BytesLeftToRead=ui32BufferSize;
	do
	{
		FD_ZERO(&l_inputFileDescriptorSet);
		FD_SET(i32FileDescriptor, &l_inputFileDescriptorSet);

		switch(::select(i32FileDescriptor+1, &l_inputFileDescriptorSet, NULL, NULL, &l_timeout))
		{
			case -1: // error
				return READ_ERROR;

			case  0: // timeout
				l_bFinished = true;
				break;

			default:
				if(FD_ISSET(i32FileDescriptor, &l_inputFileDescriptorSet))
				{
					ssize_t l_i32ReadLength=::read(i32FileDescriptor, reinterpret_cast<uint8*>(pBuffer)+ui32BufferSize-l_ui32BytesLeftToRead, l_ui32BytesLeftToRead);
					if(l_i32ReadLength <= 0)
					{
						l_bFinished = true;
					}
					else
					{
						l_ui32BytesLeftToRead-=uint32(l_i32ReadLength);
					}
				}
				else
				{
					l_bFinished = true;
				}
				break;
		}
	}
	while(!l_bFinished && l_ui32BytesLeftToRead!=0);

	return ui32BufferSize-l_ui32BytesLeftToRead;

#endif

	return 0;
}


