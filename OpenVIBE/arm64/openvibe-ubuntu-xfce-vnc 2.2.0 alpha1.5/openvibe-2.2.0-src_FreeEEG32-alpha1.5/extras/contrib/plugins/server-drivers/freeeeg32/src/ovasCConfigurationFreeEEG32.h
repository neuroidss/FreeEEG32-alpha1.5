/*
 * FreeEEG32 driver for OpenViBE
 *
 * \author Jeremy Frey
 * \author Yann Renard
 */
#ifndef __OpenViBE_AcquisitionServer_CConfigurationFreeEEG32_H__
#define __OpenViBE_AcquisitionServer_CConfigurationFreeEEG32_H__

#include "../ovasCConfigurationBuilder.h"

#include <gtk/gtk.h>
#include <map>

namespace OpenViBEAcquisitionServer
{
	class CConfigurationFreeEEG32 : public OpenViBEAcquisitionServer::CConfigurationBuilder
	{
	public:

//		const static OpenViBE::uint16 DefaultSamplingRate = 250; // sampling rate with no daisy module (divided by 2 with daisy module)
		const static OpenViBE::uint16 DefaultSamplingRate = 512; // sampling rate with no daisy module (divided by 2 with daisy module)
//		const static OpenViBE::uint16 DefaultEEGChannelCount = 8; // number of EEG channels with no daisy module (multiplied by 2 with daisy module)
		const static OpenViBE::uint16 DefaultEEGChannelCount = 32; // number of EEG channels with no daisy module (multiplied by 2 with daisy module)
		const static OpenViBE::uint16 DefaultAccChannelCount = 3; // number of Acc channels (daisy module does not have an impact)

		typedef enum
		{
			DaisyStatus_Active,
			DaisyStatus_Inactive,
		} EDaisyStatus;

		typedef struct
		{
			int iEEGChannelCount;
			int iAccChannelCount;
			int iSamplingRate;
		} SDaisyInformation;

		static OpenViBE::uint32 getMaximumTTYCount(void);
		static OpenViBE::CString getTTYFileName(const OpenViBE::uint32 ui32TTYNumber);
		static bool isTTYFile(const OpenViBE::CString& sTTYFileName);

		CConfigurationFreeEEG32(const char* sGtkBuilderFileName, OpenViBE::uint32& rUSBIndex);
		virtual ~CConfigurationFreeEEG32(void);

		virtual OpenViBE::boolean preConfigure(void);
		virtual OpenViBE::boolean postConfigure(void);

		OpenViBE::boolean setAdditionalCommands(const OpenViBE::CString& sAdditionalCommands);
		OpenViBE::CString getAdditionalCommands(void) const;

		OpenViBE::boolean setReadBoardReplyTimeout(const OpenViBE::uint32 iReadBoardReplyTimeout);
		OpenViBE::uint32 getReadBoardReplyTimeout(void) const;

		OpenViBE::boolean setFlushBoardReplyTimeout(const OpenViBE::uint32 iFlushBoardReplyTimeout);
		OpenViBE::uint32 getFlushBoardReplyTimeout(void) const;

		OpenViBE::boolean setDaisyModule(const OpenViBE::boolean sDaisyModule);
		OpenViBE::boolean getDaisyModule(void) const;

		void checkbuttonDaisyModuleCB(EDaisyStatus eStatus);

		static SDaisyInformation getDaisyInformation(EDaisyStatus eStatus);

	protected:

		std::map <int, int> m_vComboSlotIndexToSerialPort;
		OpenViBE::uint32& m_rUSBIndex;
		::GtkListStore* m_pListStore;
		::GtkEntry* l_pEntryComInit;
		OpenViBE::CString m_sAdditionalCommands;
		OpenViBE::uint32 m_iReadBoardReplyTimeout;
		OpenViBE::uint32 m_iFlushBoardReplyTimeout;
		OpenViBE::boolean m_bDaisyModule;
	};
};

#endif // __OpenViBE_AcquisitionServer_CConfigurationFreeEEG32_H__
