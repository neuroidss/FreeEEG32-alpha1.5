/*
 * FreeEEG32a1_5 driver for OpenViBE
 *
 * \author Jeremy Frey
 * \author Yann Renard
 *
 */
#include "ovasCConfigurationFreeEEG32a1_5.h"
#include <algorithm>
#include <string>

#if defined TARGET_OS_Windows
 #include <windows.h>
 #include <winbase.h>
 #include <cstdio>
 #include <cstdlib>
 #include <commctrl.h>
 #include <winsock2.h> // htons and co.
 //#define TERM_SPEED 57600
// #define TERM_SPEED CBR_115200 // FreeEEG32a1_5 is a bit faster than others
// #define TERM_SPEED 930232 // FreeEEG32a1_5 is a bit faster than others
 #define TERM_SPEED 921600 // FreeEEG32a1_5 is a bit faster than others
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
// #define TERM_SPEED 930232
 #define TERM_SPEED 921600
#else
#endif

#define boolean OpenViBE::boolean
using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBEAcquisitionServer;

#define MAXIMUM_SERIAL_TTY     (32)
#define MAXIMUM_SERIAL_USB_TTY (256-MAXIMUM_SERIAL_TTY)

uint32 OpenViBEAcquisitionServer::CConfigurationFreeEEG32a1_5::getMaximumTTYCount(void)
{
	return MAXIMUM_SERIAL_USB_TTY + MAXIMUM_SERIAL_TTY;
}

CString OpenViBEAcquisitionServer::CConfigurationFreeEEG32a1_5::getTTYFileName(const uint32 ui32TTYNumber)
{
	char l_sBuffer[1024];
#if defined TARGET_OS_Windows
	::sprintf(l_sBuffer, "\\\\.\\COM%u", ui32TTYNumber);
#elif defined TARGET_OS_Linux
	if(ui32TTYNumber<MAXIMUM_SERIAL_USB_TTY)
	{
		::sprintf(l_sBuffer, "/dev/ttyUSB%u", ui32TTYNumber);
	}
	else
	{
		::sprintf(l_sBuffer, "/dev/ttyS%u", ui32TTYNumber-MAXIMUM_SERIAL_USB_TTY);
	}
#else
	::sprintf(l_sBuffer, "");
#endif
	return l_sBuffer;
}

bool OpenViBEAcquisitionServer::CConfigurationFreeEEG32a1_5::isTTYFile(const CString& sFileName)
{
#if defined TARGET_OS_Windows
	HANDLE l_pFile=::CreateFile(
		(LPCSTR)sFileName,
		GENERIC_READ,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);
	if(l_pFile == INVALID_HANDLE_VALUE || l_pFile == NULL)
	{
		return false;
	}
	CloseHandle(l_pFile);
	return true;
#elif defined TARGET_OS_Linux
	int l_iFile=::open(sFileName, O_RDONLY);
	if(l_iFile < 0)
	{
		return false;
	}
	close(l_iFile);
	return true;
#else
	// Caution, this path will claim all serial ports do exist because there was no platform specific implementation
	return true;
#endif
}
static void checkbutton_daisy_module_cb(::GtkToggleButton* pButton, CConfigurationFreeEEG32a1_5* pConfigurationFreeEEG32a1_5)
{
	pConfigurationFreeEEG32a1_5->checkbuttonDaisyModuleCB(gtk_toggle_button_get_active(pButton)?CConfigurationFreeEEG32a1_5::DaisyStatus_Active:CConfigurationFreeEEG32a1_5::DaisyStatus_Inactive);
}

CConfigurationFreeEEG32a1_5::CConfigurationFreeEEG32a1_5(const char* sGtkBuilderFileName, OpenViBE::uint32& rUSBIndex)
	:CConfigurationBuilder(sGtkBuilderFileName)
	,m_rUSBIndex(rUSBIndex)
{
	m_pListStore=gtk_list_store_new(1, G_TYPE_STRING);
}

CConfigurationFreeEEG32a1_5::~CConfigurationFreeEEG32a1_5(void)
{
	g_object_unref(m_pListStore);
}

boolean CConfigurationFreeEEG32a1_5::preConfigure(void)
{
	if(!CConfigurationBuilder::preConfigure())
	{
		return false;
	}

#if 0
	::GtkEntry* l_pEntryComInit=GTK_ENTRY(gtk_builder_get_object(m_pBuilderConfigureInterface, "entry_com_init"));
	::gtk_entry_set_text(l_pEntryComInit, m_sAdditionalCommands.toASCIIString());
#else
	std::string l_sAdditionalCommands=m_sAdditionalCommands.toASCIIString();
	std::replace(l_sAdditionalCommands.begin(), l_sAdditionalCommands.end(), '\255', '\n');
	::GtkTextView* l_pTextViewComInit=GTK_TEXT_VIEW(gtk_builder_get_object(m_pBuilderConfigureInterface, "text_view_com_init"));
	::GtkTextBuffer* l_pTextBufferComInit=::gtk_text_view_get_buffer(l_pTextViewComInit);
	::gtk_text_buffer_set_text(l_pTextBufferComInit, l_sAdditionalCommands.c_str(), -1);
#endif

	::GtkSpinButton* l_pSpinButtonReadBoardReplyTimeout=GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "spinbutton_read_board_reply_timeout"));
	::gtk_spin_button_set_value(l_pSpinButtonReadBoardReplyTimeout, m_iReadBoardReplyTimeout);

	::GtkSpinButton* l_pSpinButtonFlushBoardReplyTimeout=GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "spinbutton_flush_board_reply_timeout"));
	::gtk_spin_button_set_value(l_pSpinButtonFlushBoardReplyTimeout, m_iFlushBoardReplyTimeout);

	::GtkToggleButton* l_pToggleButtonDaisyModule=GTK_TOGGLE_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "checkbutton_daisy_module"));
	::gtk_toggle_button_set_active(l_pToggleButtonDaisyModule, m_bDaisyModule?true:false);

	::g_signal_connect(::gtk_builder_get_object(m_pBuilderConfigureInterface, "checkbutton_daisy_module"), "toggled", G_CALLBACK(checkbutton_daisy_module_cb), this);
	this->checkbuttonDaisyModuleCB(m_bDaisyModule?DaisyStatus_Active:DaisyStatus_Inactive);

	::GtkComboBox* l_pComboBox=GTK_COMBO_BOX(gtk_builder_get_object(m_pBuilderConfigureInterface, "combobox_device"));

	g_object_unref(m_pListStore);
	m_pListStore=gtk_list_store_new(1, G_TYPE_STRING);
	m_vComboSlotIndexToSerialPort.clear();

	gtk_combo_box_set_model(l_pComboBox, GTK_TREE_MODEL(m_pListStore));

	boolean l_bSelected=false;

	m_vComboSlotIndexToSerialPort[0]=-1;
	::gtk_combo_box_append_text(l_pComboBox, "Automatic");

	for(uint32 i=0, j=1; i<CConfigurationFreeEEG32a1_5::getMaximumTTYCount(); i++)
	{
		CString l_sFileName = CConfigurationFreeEEG32a1_5::getTTYFileName(i);
		if(CConfigurationFreeEEG32a1_5::isTTYFile(l_sFileName))
		{
			m_vComboSlotIndexToSerialPort[j]=i;
			::gtk_combo_box_append_text(l_pComboBox, l_sFileName.toASCIIString());
			if(m_rUSBIndex==i)
			{
				::gtk_combo_box_set_active(l_pComboBox, j);
				l_bSelected=true;
			}
			j++;
		}
	}

	if(!l_bSelected)
	{
		::gtk_combo_box_set_active(l_pComboBox, 0);
	}

	return true;
}

boolean CConfigurationFreeEEG32a1_5::postConfigure(void)
{
	::GtkComboBox* l_pComboBox=GTK_COMBO_BOX(gtk_builder_get_object(m_pBuilderConfigureInterface, "combobox_device"));

	if(m_bApplyConfiguration)
	{
		int l_iUSBIndex=m_vComboSlotIndexToSerialPort[gtk_combo_box_get_active(l_pComboBox)];
		if(l_iUSBIndex>=0)
		{
			m_rUSBIndex=(uint32)l_iUSBIndex;
		}
		else
		{
			m_rUSBIndex=(uint32)-1;
		}

#if 0
		::GtkEntry* l_pEntryComInit=GTK_ENTRY(gtk_builder_get_object(m_pBuilderConfigureInterface, "entry_com_init"));
		m_sAdditionalCommands=::gtk_entry_get_text(l_pEntryComInit);
#else
		::GtkTextView* l_pTextViewComInit=GTK_TEXT_VIEW(gtk_builder_get_object(m_pBuilderConfigureInterface, "text_view_com_init"));
		::GtkTextBuffer* l_pTextBufferComInit=::gtk_text_view_get_buffer(l_pTextViewComInit);
		::GtkTextIter l_oStartIter;
		::GtkTextIter l_oEndIter;
		::gtk_text_buffer_get_start_iter(l_pTextBufferComInit, &l_oStartIter);
		::gtk_text_buffer_get_end_iter(l_pTextBufferComInit, &l_oEndIter);
		std::string l_sAdditionalCommands=::gtk_text_buffer_get_text(l_pTextBufferComInit, &l_oStartIter, &l_oEndIter, FALSE);
		std::replace(l_sAdditionalCommands.begin(), l_sAdditionalCommands.end(), '\n', '\255');
		m_sAdditionalCommands = l_sAdditionalCommands.c_str();
#endif

		::GtkSpinButton* l_pSpinButtonReadBoardReplyTimeout=GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "spinbutton_read_board_reply_timeout"));
		gtk_spin_button_update(GTK_SPIN_BUTTON(l_pSpinButtonReadBoardReplyTimeout));
		m_iReadBoardReplyTimeout=::gtk_spin_button_get_value_as_int(l_pSpinButtonReadBoardReplyTimeout);

		::GtkSpinButton* l_pSpinButtonFlushBoardReplyTimeout=GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "spinbutton_flush_board_reply_timeout"));
		gtk_spin_button_update(GTK_SPIN_BUTTON(l_pSpinButtonFlushBoardReplyTimeout));
		m_iFlushBoardReplyTimeout=::gtk_spin_button_get_value_as_int(l_pSpinButtonFlushBoardReplyTimeout);

		::GtkToggleButton* l_pToggleButtonDaisyModule=GTK_TOGGLE_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "checkbutton_daisy_module"));
		m_bDaisyModule=::gtk_toggle_button_get_active(l_pToggleButtonDaisyModule)?true:false;
	}

	if(!CConfigurationBuilder::postConfigure())
	{
		return false;
	}
	return true;
}

boolean CConfigurationFreeEEG32a1_5::setAdditionalCommands(const CString& sAdditionalCommands)
{
        m_sAdditionalCommands=sAdditionalCommands;
        return true;
}

CString CConfigurationFreeEEG32a1_5::getAdditionalCommands(void) const
{
        return m_sAdditionalCommands;
}

boolean CConfigurationFreeEEG32a1_5::setReadBoardReplyTimeout(uint32 iReadBoardReplyTimeout)
{
        m_iReadBoardReplyTimeout=iReadBoardReplyTimeout;
        return true;
}

uint32 CConfigurationFreeEEG32a1_5::getReadBoardReplyTimeout(void) const
{
        return m_iReadBoardReplyTimeout;
}

boolean CConfigurationFreeEEG32a1_5::setFlushBoardReplyTimeout(uint32 iFlushBoardReplyTimeout)
{
        m_iFlushBoardReplyTimeout=iFlushBoardReplyTimeout;
        return true;
}

uint32 CConfigurationFreeEEG32a1_5::getFlushBoardReplyTimeout(void) const
{
        return m_iFlushBoardReplyTimeout;
}

bool CConfigurationFreeEEG32a1_5::setDaisyModule(bool bDaisyModule)
{
        m_bDaisyModule=bDaisyModule;
        return true;
}

bool CConfigurationFreeEEG32a1_5::getDaisyModule(void) const
{
        return m_bDaisyModule;
}

void CConfigurationFreeEEG32a1_5::checkbuttonDaisyModuleCB(EDaisyStatus eStatus)
{
	SDaisyInformation l_oDaisyInformation=this->getDaisyInformation(eStatus);

	char l_sBuffer[1024];

	::sprintf(l_sBuffer, "%i EEG Channels", l_oDaisyInformation.iEEGChannelCount);
	::gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(m_pBuilderConfigureInterface, "label_status_eeg_channel_count")), l_sBuffer);

	::sprintf(l_sBuffer, "%i Accelerometer Channels", l_oDaisyInformation.iAccChannelCount);
	::gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(m_pBuilderConfigureInterface, "label_status_acc_channel_count")), l_sBuffer);

	::sprintf(l_sBuffer, "%i Hz Sampling Rate", l_oDaisyInformation.iSamplingRate);
	::gtk_label_set_text(GTK_LABEL(gtk_builder_get_object(m_pBuilderConfigureInterface, "label_status_sampling_rate")), l_sBuffer);

	::gtk_spin_button_set_value(GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderConfigureInterface, "spinbutton_number_of_channels")), l_oDaisyInformation.iEEGChannelCount+l_oDaisyInformation.iAccChannelCount);
}

CConfigurationFreeEEG32a1_5::SDaisyInformation CConfigurationFreeEEG32a1_5::getDaisyInformation(EDaisyStatus eStatus)
{
	SDaisyInformation l_oResult;
	switch(eStatus)
	{
		case DaisyStatus_Inactive:
			l_oResult.iEEGChannelCount = DefaultEEGChannelCount;
			l_oResult.iAccChannelCount = DefaultAccChannelCount;
			l_oResult.iSamplingRate = DefaultSamplingRate;
			break;

		case DaisyStatus_Active:
			l_oResult.iEEGChannelCount = DefaultEEGChannelCount*2;
			l_oResult.iAccChannelCount = DefaultAccChannelCount;
			l_oResult.iSamplingRate = DefaultSamplingRate/2;
			break;

		default:
			l_oResult.iEEGChannelCount = 0;
			l_oResult.iAccChannelCount = 0;
			l_oResult.iSamplingRate = 0;
			break;
	}

	return l_oResult;
}

