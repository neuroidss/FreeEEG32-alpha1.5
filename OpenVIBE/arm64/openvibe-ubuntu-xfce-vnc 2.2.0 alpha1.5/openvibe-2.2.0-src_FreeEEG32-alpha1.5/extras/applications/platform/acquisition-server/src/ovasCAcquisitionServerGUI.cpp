#include "ovasCAcquisitionServerGUI.h"
#include "ovasCAcquisitionServerThread.h"
#include "ovasCAcquisitionServer.h"
#include "ovasIAcquisitionServerPlugin.h"
// Drivers

#ifdef TARGET_HAS_OpenViBEContributions
#include "contribAcquisitionServer.inl"
#endif

#include "ovasCPluginLSLOutput.h"
#include "ovasCPluginFiddler.h"

#include "generic-oscillator/ovasCDriverGenericOscillator.h"
#include "generic-sawtooth/ovasCDriverGenericSawTooth.h"
#include "generic-time-signal/ovasCDriverGenericTimeSignal.h"
#include "generic-raw-reader/ovasCDriverGenericRawFileReader.h"
#include "generic-raw-reader/ovasCDriverGenericRawTelnetReader.h"

#include "biosemi-activetwo/ovasCDriverBioSemiActiveTwo.h"
#include "brainproducts-actichamp/ovasCDriverBrainProductsActiCHamp.h"
#include "brainproducts-brainampseries/ovasCDriverBrainProductsBrainampSeries.h"
#include "brainproducts-vamp/ovasCDriverBrainProductsVAmp.h"
#include "brainproducts-liveamp/ovasCDriverBrainProductsLiveAmp.h"
#include "egi-ampserver/ovasCDriverEGIAmpServer.h"
#include "emotiv-epoc/ovasCDriverEmotivEPOC.h"
#include "labstreaminglayer/ovasCDriverLabStreamingLayer.h"
#include "micromed-systemplusevolution/ovasCDriverMicromedSystemPlusEvolution.h"
#include "mindmedia-nexus32b/ovasCDriverMindMediaNeXus32B.h"
#include "mcs-nvx/ovasCDriverMCSNVXDriver.h"
#include "neuroelectrics-enobio3g/ovasCDriverEnobio3G.h"
#include "neuroservo/ovasCDriverNeuroServoHid.h"
#include "neurosky-mindset/ovasCDriverNeuroskyMindset.h"
#include "tmsi/ovasCDriverTMSi.h"
#include "tmsi-refa32b/ovasCDriverTMSiRefa32B.h"

#include "mensia-acquisition/ovasCDriverMensiaAcquisition.h"

#include <system/ovCMemory.h>
#include <system/ovCTime.h>
#include <limits>

#include <toolkit/ovtk_all.h>

// Plugins



#include <fstream>
#include <sstream>

#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <cctype>
#include <cstring>

#include <cassert>
#include <system/WindowsUtilities.h>
//

#define boolean OpenViBE::boolean

namespace
{
	// because std::tolower has multiple signatures,
	// it can not be easily used in std::transform
	// this workaround is taken from http://www.gcek.net/ref/books/sw/cpp/ticppv2/
	template <class charT>
	charT to_lower(charT c)
	{
		return std::tolower(c);
	}
};

#define OVAS_GUI_File            OpenViBE::Directories::getDataDir() + "/applications/acquisition-server/interface.ui"

using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBEAcquisitionServer;
using namespace std;

//___________________________________________________________________//
//                                                                   //

static void button_preference_pressed_cb(::GtkButton* pButton, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->buttonPreferencePressedCB(pButton);
}

static void button_configure_pressed_cb(::GtkButton* pButton, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->buttonConfigurePressedCB(pButton);
}

static void button_connect_toggled_cb(::GtkToggleButton* pButton, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->buttonConnectToggledCB(pButton);
}

static void button_start_pressed_cb(::GtkButton* pButton, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->buttonStartPressedCB(pButton);
}

static void button_stop_pressed_cb(::GtkButton* pButton, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->buttonStopPressedCB(pButton);
}

static void combobox_driver_changed_cb(::GtkComboBox* pComboBox, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->comboBoxDriverChanged(pComboBox);
}

static void combobox_sample_count_per_sent_block_changed_cb(::GtkComboBox* pComboBox, void* pUserData)
{
	static_cast<CAcquisitionServerGUI*>(pUserData)->comboBoxSampleCountPerSentBlockChanged(pComboBox);
}

static bool compare_driver_names(OpenViBEAcquisitionServer::IDriver* a,
	OpenViBEAcquisitionServer::IDriver* b)
{
	std::string l_sA = a->getName();
	std::string l_sB = b->getName();

	std::transform(l_sA.begin(), l_sA.end(), l_sA.begin(), ::tolower);
	std::transform(l_sB.begin(), l_sB.end(), l_sB.begin(), ::tolower);

	return l_sA < l_sB;
}


//___________________________________________________________________//
//                                                                   //

CAcquisitionServerGUI::CAcquisitionServerGUI(const IKernelContext& rKernelContext)
	:m_rKernelContext(rKernelContext)
	,m_pDriver(NULL)
	,m_pDriverContext(NULL)
	,m_pAcquisitionServer(NULL)
	,m_pAcquisitionServerThread(NULL)
	,m_pBuilderInterface(NULL)
	,m_pImpedanceWindow(NULL)
	,m_pThread(NULL)
{
	// boolean l_bShowUnstable=m_rKernelContext.getConfigurationManager().expandAsBoolean("${AcquisitionServer_ShowUnstable}", false);

	m_pAcquisitionServer=new CAcquisitionServer(rKernelContext);

	m_vDriver.push_back(new CDriverGenericOscillator(m_pAcquisitionServer->getDriverContext()));
	m_vDriver.push_back(new CDriverGenericSawTooth(m_pAcquisitionServer->getDriverContext()));
	m_vDriver.push_back(new CDriverGenericTimeSignal(m_pAcquisitionServer->getDriverContext()));
	m_vDriver.push_back(new CDriverGenericRawFileReader(m_pAcquisitionServer->getDriverContext()));
	m_vDriver.push_back(new CDriverGenericRawTelnetReader(m_pAcquisitionServer->getDriverContext()));

#if defined TARGET_OS_Windows
	m_vDriver.push_back(new CDriverBrainProductsBrainampSeries(m_pAcquisitionServer->getDriverContext()));
#endif

#if defined TARGET_HAS_ThirdPartyActiCHampAPI
	m_vDriver.push_back(new CDriverBrainProductsActiCHamp(m_pAcquisitionServer->getDriverContext()));
#endif

#if defined TARGET_HAS_ThirdPartyLiveAmpAPI
	m_vDriver.push_back(new CDriverBrainProductsLiveAmp(m_pAcquisitionServer->getDriverContext()));
#endif

#if defined TARGET_HAS_ThirdPartyBioSemiAPI
	m_vDriver.push_back(new CDriverBioSemiActiveTwo(m_pAcquisitionServer->getDriverContext()));
#endif

	m_vDriver.push_back(new CDriverEGIAmpServer(m_pAcquisitionServer->getDriverContext()));

#if defined TARGET_HAS_ThirdPartyEmotivAPI
	m_vDriver.push_back(new CDriverEmotivEPOC(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyEnobioAPI
	m_vDriver.push_back(new CDriverEnobio3G(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyMCS
	m_vDriver.push_back(new CDriverMKSNVXDriver(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined(TARGET_HAS_ThirdPartyMicromed)
	m_vDriver.push_back(new CDriverMicromedSystemPlusEvolution(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined(TARGET_HAS_ThirdPartyNeuroServo)
	m_vDriver.push_back(new CDriverNeuroServoHid(m_pAcquisitionServer->getDriverContext()));
#endif
//#if defined TARGET_HAS_ThirdPartyEEGOAPI
//    m_vDriver.push_back(new CDriverEEGO(m_pAcquisitionServer->getDriverContext()));
//#endif


#if defined(TARGET_HAS_ThirdPartyNeXus)
	m_vDriver.push_back(new CDriverMindMediaNeXus32B(m_pAcquisitionServer->getDriverContext()));
	m_vDriver.push_back(new CDriverTMSiRefa32B(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyThinkGearAPI
	m_vDriver.push_back(new CDriverNeuroskyMindset(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyTMSi
	m_vDriver.push_back(new CDriverTMSi(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyUSBFirstAmpAPI
	m_vDriver.push_back(new CDriverBrainProductsVAmp(m_pAcquisitionServer->getDriverContext()));
#endif
#if defined TARGET_HAS_ThirdPartyLSL
	m_vDriver.push_back(new CDriverLabStreamingLayer(m_pAcquisitionServer->getDriverContext()));
#endif

	// BEGIN MENSIA ACQUISITION DRIVERS
#if defined TARGET_OS_Windows && defined TARGET_HasMensiaAcquisitionDriver

	m_pAcquisitionServer->getDriverContext().getLogManager() << LogLevel_Trace << "Loading Mensia Driver Collection\n";
	m_pLibMensiaAcquisition = nullptr;
	CString l_sMensiaDLLPath = m_pAcquisitionServer->getDriverContext().getConfigurationManager().expand("${Path_Bin}/openvibe-driver-mensia-acquisition.dll");
	if(!std::ifstream(l_sMensiaDLLPath.toASCIIString()).is_open())
	{
		m_pAcquisitionServer->getDriverContext().getLogManager() << LogLevel_Trace << "Couldn't open" <<
			" dll file [" << l_sMensiaDLLPath.toASCIIString() <<"], perhaps it was not installed.\n";
	}
	else
	{
		m_pLibMensiaAcquisition = System::WindowsUtilities::utf16CompliantLoadLibrary(l_sMensiaDLLPath.toASCIIString());
		HINSTANCE l_oLibMensiaAcquisition = static_cast<HINSTANCE>(m_pLibMensiaAcquisition);

		//if it can't be open return FALSE;
		if( l_oLibMensiaAcquisition == NULL)
		{
			m_pAcquisitionServer->getDriverContext().getLogManager() << LogLevel_Warning << "Couldn't load DLL: [" << l_sMensiaDLLPath << "]. Got error: [" << static_cast<uint64>(GetLastError()) << "]\n";
		}
		else
		{
			typedef int32 (*MACQ_InitializeMensiaAcquisitionLibrary)();
			MACQ_InitializeMensiaAcquisitionLibrary l_fpInitializeMensiaAcquisitionLibrary;
			l_fpInitializeMensiaAcquisitionLibrary = (MACQ_InitializeMensiaAcquisitionLibrary)::GetProcAddress(l_oLibMensiaAcquisition, "initializeAcquisitionLibrary");
			typedef const char* (*MACQ_GetDriverId)(unsigned int uiDriverId);
			MACQ_GetDriverId l_fpGetDriverID;
			l_fpGetDriverID = (MACQ_GetDriverId)::GetProcAddress(l_oLibMensiaAcquisition, "getDriverId");

			int32 l_i32MensiaDeviceCount = l_fpInitializeMensiaAcquisitionLibrary();

			if (l_i32MensiaDeviceCount >= 0)
			{
				for (size_t l_uiDeviceIndex = 0; l_uiDeviceIndex < static_cast<uint32>(l_i32MensiaDeviceCount); l_uiDeviceIndex++)
				{
					char l_sDriverIdentifier[1024];

					strcpy(l_sDriverIdentifier, l_fpGetDriverID(l_uiDeviceIndex));
					if (strcmp(l_sDriverIdentifier, "") != 0)
					{
						m_pAcquisitionServer->getDriverContext().getLogManager() << LogLevel_Info << "Found driver [" << l_sDriverIdentifier << "] in Mensia Driver Collection" << "\n";
						m_vDriver.push_back(new CDriverMensiaAcquisition(m_pAcquisitionServer->getDriverContext(), l_sDriverIdentifier));
					}

				}
			}
			else
			{
				m_pAcquisitionServer->getDriverContext().getLogManager() << LogLevel_Error << "Error occurred while initializing Mensia Acquisition Library" << "\n";
			}

			::FreeLibrary(l_oLibMensiaAcquisition);
		}
	}
#endif
	// END MENSIA ACQUISITION DRIVERS

#if defined TARGET_HAS_OpenViBEContributions
	OpenViBEContributions::initiateContributions(this, m_pAcquisitionServer, rKernelContext, &m_vDriver);
#endif

	// Plugins that just send out data must be the last in list (since other plugins may modify the data)

#if defined TARGET_HAS_ThirdPartyLSL
	registerPlugin(new OpenViBEAcquisitionServer::OpenViBEAcquisitionServerPlugins::CPluginLSLOutput(rKernelContext));
#endif

	registerPlugin(new OpenViBEAcquisitionServer::OpenViBEAcquisitionServerPlugins::CPluginFiddler(rKernelContext));

	std::sort(m_vDriver.begin(), m_vDriver.end(), compare_driver_names);

	scanPluginSettings();

	m_pAcquisitionServerThread=new CAcquisitionServerThread(m_rKernelContext, *this, *m_pAcquisitionServer);

	// Initialize GTK objects as the thread started below may refer to them quickly
	this->initialize();

	m_pThread=new std::thread(CAcquisitionServerThreadHandle(*m_pAcquisitionServerThread));
}

CAcquisitionServerGUI::~CAcquisitionServerGUI(void)
{
	m_pAcquisitionServerThread->terminate();
	m_pThread->join();

	savePluginSettings();

	// Saves current configuration
	FILE* l_pFile=::fopen(m_rKernelContext.getConfigurationManager().expand("${Path_UserData}/openvibe-acquisition-server.conf").toASCIIString(), "wt");
	if(l_pFile)
	{
		::fprintf(l_pFile, "# This file is generated\n");
		::fprintf(l_pFile, "# Do not modify\n");
		::fprintf(l_pFile, "\n");
		::fprintf(l_pFile, "# Last settings set in the acquisition server\n");
		::fprintf(l_pFile, "AcquisitionServer_LastDriver = %s\n", m_pDriver->getName());
		::fprintf(l_pFile, "AcquisitionServer_LastSampleCountPerBuffer = %i\n", this->getSampleCountPerBuffer());
		::fprintf(l_pFile, "AcquisitionServer_LastConnectionPort = %i\n", this->getTCPPort());
		::fprintf(l_pFile, "# Last Preferences set in the acquisition server\n");
		::fprintf(l_pFile, "AcquisitionServer_DriftCorrectionPolicy = %s\n", m_pAcquisitionServer->m_oDriftCorrection.getDriftCorrectionPolicyStr().toASCIIString());
		::fprintf(l_pFile, "AcquisitionServer_JitterEstimationCountForDrift = %u\n", static_cast<unsigned int>(m_pAcquisitionServer->m_oDriftCorrection.getJitterEstimationCountForDrift()));
		::fprintf(l_pFile, "AcquisitionServer_DriftToleranceDuration = %u\n", static_cast<unsigned int>(m_pAcquisitionServer->m_oDriftCorrection.getDriftToleranceDurationMs()));
		::fprintf(l_pFile, "AcquisitionServer_OverSamplingFactor = %u\n", static_cast<unsigned int>(m_pAcquisitionServer->getOversamplingFactor()));
		::fprintf(l_pFile, "AcquisitionServer_ChannelSelection = %s\n", (m_pAcquisitionServer->isChannelSelectionRequested() ? "True" : "False"));
		::fprintf(l_pFile, "AcquisitionServer_NaNReplacementPolicy = %s\n", m_pAcquisitionServer->getNaNReplacementPolicyStr().toASCIIString());

		::fprintf(l_pFile, "# Settings for various device drivers\n");
		std::vector<std::string> l_vTokens;
		IConfigurationManager* l_pConfigurationManager = &m_rKernelContext.getConfigurationManager();
		CIdentifier l_oTokenIdentifier; // defaults to OV_UndefinedIdentifier
		// Collect token names
		while((l_oTokenIdentifier=l_pConfigurationManager->getNextConfigurationTokenIdentifier(l_oTokenIdentifier)) != OV_UndefinedIdentifier)
		{
			const std::string prefix("AcquisitionServer_Driver_");
			const std::string prefix2("AcquisitionServer_Plugin_");
			CString tokenName = l_pConfigurationManager->getConfigurationTokenName(l_oTokenIdentifier);
			if(std::string(tokenName.toASCIIString()).compare(0,prefix.length(), prefix) == 0 ||
				std::string(tokenName.toASCIIString()).compare(0,prefix.length(), prefix2) == 0)
			{
				std::string token = std::string(tokenName.toASCIIString());
				l_vTokens.push_back(token);
			}
		}
		std::sort(l_vTokens.begin(), l_vTokens.end());
		// Write out as sorted
		for(size_t i=0;i<l_vTokens.size();i++) {
			CString tokenValue = l_pConfigurationManager->lookUpConfigurationTokenValue(l_vTokens[i].c_str());

			::fprintf(l_pFile, "%s = %s\n", l_vTokens[i].c_str(), tokenValue.toASCIIString());
		}

		::fclose(l_pFile);
	}

	for(auto itDriver=m_vDriver.begin(); itDriver!=m_vDriver.end(); itDriver++)
	{
		m_rKernelContext.getLogManager() << LogLevel_Debug << "Deleting " << (*itDriver)->getName() << "\n";
		delete (*itDriver);
	}

	m_vDriver.clear();
	m_pDriver=NULL;

	// BEGIN MENSIA ACQUISITION DRIVERS
	// For future implementation
#if defined TARGET_OS_Windows && defined TARGET_HasMensiaAcquisitionDriver
	typedef int32 (*MACQ_ReleaseMensiaAcquisitionLibrary)();
	MACQ_ReleaseMensiaAcquisitionLibrary l_fpReleaseMensiaAcquisitionLibrary;
	l_fpReleaseMensiaAcquisitionLibrary = (MACQ_ReleaseMensiaAcquisitionLibrary)::GetProcAddress(static_cast<HINSTANCE>(m_pLibMensiaAcquisition), "releaseAcquisitionLibrary");
//	l_fpReleaseMensiaAcquisitionLibrary();
#endif
	// END MENSIA ACQUISITION DRIVERS

	delete m_pThread;
	m_pThread=NULL;

	delete m_pAcquisitionServerThread;
	m_pAcquisitionServerThread=NULL;

	delete m_pAcquisitionServer;
	m_pAcquisitionServer=NULL;
}

//___________________________________________________________________//
//                                                                   //

boolean CAcquisitionServerGUI::initialize(void)
{
	m_pBuilderInterface=gtk_builder_new(); // glade_xml_new(OVAS_GUI_File, NULL, NULL);
	gtk_builder_add_from_file(m_pBuilderInterface, OVAS_GUI_File, NULL);

	// Connects custom GTK signals

	// Note: Seems the signals below have to be "clicked", not "pressed", or the underlined keyboard shortcuts
	// of gtk stock items that can be activated with alt key ("mnemonics") do not work.
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "button_preference"),                    "clicked", G_CALLBACK(button_preference_pressed_cb), this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "button_configure"),                     "clicked", G_CALLBACK(button_configure_pressed_cb),  this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "togglebutton_connect"),                 "toggled", G_CALLBACK(button_connect_toggled_cb),    this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "button_play"),                          "clicked", G_CALLBACK(button_start_pressed_cb),      this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "button_stop"),                          "clicked", G_CALLBACK(button_stop_pressed_cb),       this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "combobox_driver"),                      "changed", G_CALLBACK(combobox_driver_changed_cb),   this);
	g_signal_connect(gtk_builder_get_object(m_pBuilderInterface, "combobox_sample_count_per_sent_block"), "changed", G_CALLBACK(combobox_sample_count_per_sent_block_changed_cb),  this);
	gtk_builder_connect_signals(m_pBuilderInterface, NULL);

	::GtkComboBox* l_pComboBoxDriver=GTK_COMBO_BOX(gtk_builder_get_object(m_pBuilderInterface, "combobox_driver"));

	enum
	{
		Resource_StringMarkup,
	};

	// Prepares drivers combo box

	gtk_combo_box_set_model(l_pComboBoxDriver, NULL);

	::GtkCellRenderer* l_pCellRendererName=gtk_cell_renderer_text_new();

	gtk_cell_layout_clear(GTK_CELL_LAYOUT(l_pComboBoxDriver));
	gtk_cell_layout_pack_start(GTK_CELL_LAYOUT(l_pComboBoxDriver), l_pCellRendererName, TRUE);
	gtk_cell_layout_add_attribute(GTK_CELL_LAYOUT(l_pComboBoxDriver), l_pCellRendererName, "markup", Resource_StringMarkup);

	::GtkTreeStore* l_pDriverTreeStore=gtk_tree_store_new(1, G_TYPE_STRING);
	gtk_combo_box_set_model(l_pComboBoxDriver, GTK_TREE_MODEL(l_pDriverTreeStore));

	string l_sDefaultDriverName=m_rKernelContext.getConfigurationManager().expand("${AcquisitionServer_DefaultDriver}").toASCIIString();
	transform(l_sDefaultDriverName.begin(), l_sDefaultDriverName.end(), l_sDefaultDriverName.begin(), ::to_lower<string::value_type>);
	for(size_t i=0; i<m_vDriver.size(); i++) // n.b. dont use iterator here as we need a numeric index for gtk later anyway
	{
		IDriver *l_pDriver = m_vDriver[i];

		::GtkTreeIter l_oIter;
		gtk_tree_store_append(l_pDriverTreeStore, &l_oIter, NULL);

		string l_sDriverName=l_pDriver->getName();

		const bool l_bUnstable = l_pDriver->isFlagSet(DriverFlag_IsUnstable);
		const bool l_bDeprecated = l_pDriver->isFlagSet(DriverFlag_IsDeprecated);

		const std::string l_sStringToDisplay =
			   std::string((l_bUnstable || l_bDeprecated) ? "<span foreground=\"#6f6f6f\">" : "")
			+= l_sDriverName 
			+= std::string((l_bUnstable || l_bDeprecated) ? "</span>" : "")
			+= std::string(l_bUnstable ?  " <span size=\"smaller\" style=\"italic\">(<span foreground=\"#202060\">unstable</span>)</span>" : "")
			+= std::string(l_bDeprecated ?  " <span size=\"smaller\" style=\"italic\">(<span foreground=\"#602020\">deprecated</span>)</span>" : "");

		gtk_tree_store_set(l_pDriverTreeStore, &l_oIter,
			Resource_StringMarkup, l_sStringToDisplay.c_str(), -1);

		transform(l_sDriverName.begin(), l_sDriverName.end(), l_sDriverName.begin(), ::to_lower<string::value_type>);
		if(l_sDefaultDriverName==l_sDriverName)
		{
			gtk_combo_box_set_active(l_pComboBoxDriver, i);
		}
	}
	if(gtk_combo_box_get_active(l_pComboBoxDriver)==-1)
	{
		gtk_combo_box_set_active(l_pComboBoxDriver, 0);
	}

	// Prepares sample count per buffer combo box

	boolean l_bFound=false;
	string l_sDefaultSampleCountPerBuffer=m_rKernelContext.getConfigurationManager().expand("${AcquisitionServer_DefaultSampleCountPerBuffer}").toASCIIString();
	::GtkComboBox* l_pComboBoxSampleCountPerBuffer=GTK_COMBO_BOX(gtk_builder_get_object(m_pBuilderInterface, "combobox_sample_count_per_sent_block"));
	for(int i=0; ; i++)
	{
		gtk_combo_box_set_active(l_pComboBoxSampleCountPerBuffer, i);
		if(gtk_combo_box_get_active(l_pComboBoxSampleCountPerBuffer)==-1)
		{
			break;
		}
		if(l_sDefaultSampleCountPerBuffer==gtk_combo_box_get_active_text(l_pComboBoxSampleCountPerBuffer))
		{
			l_bFound=true;
			break;
		}
	}
	if(!l_bFound)
	{
		if(l_sDefaultSampleCountPerBuffer != "-1")
		{
			gtk_combo_box_prepend_text(l_pComboBoxSampleCountPerBuffer, l_sDefaultSampleCountPerBuffer.c_str());
		}
		gtk_combo_box_set_active(l_pComboBoxSampleCountPerBuffer, 0);
	}

	// Prepares default connection port

	::GtkSpinButton* l_pSpinButtonConnectionPort=GTK_SPIN_BUTTON(gtk_builder_get_object(m_pBuilderInterface, "spinbutton_connection_port"));
	uint64 l_ui64DefaultConnectionPort=m_rKernelContext.getConfigurationManager().expandAsUInteger("${AcquisitionServer_DefaultConnectionPort}", 1024);
	gtk_spin_button_set_value(l_pSpinButtonConnectionPort, (gdouble)l_ui64DefaultConnectionPort);

	// Optionnally autostarts

	if(m_rKernelContext.getConfigurationManager().expandAsBoolean("${AcquisitionServer_AutoStart}", false))
	{
		::gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(::gtk_builder_get_object(m_pBuilderInterface, "togglebutton_connect")), TRUE);
//		::gtk_button_pressed(GTK_BUTTON(::gtk_builder_get_object(m_pBuilderInterface, "button_play")));
		::gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(::gtk_builder_get_object(m_pBuilderInterface, "button_play")), TRUE);
	}


	// Shows main window
	if(!m_rKernelContext.getConfigurationManager().expandAsBoolean("${AcquisitionServer_NoGUI}", false))
	{
		gtk_widget_show(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "openvibe-acquisition-server")));
	}

	return true;
}

//___________________________________________________________________//
//                                                                   //

IDriver& CAcquisitionServerGUI::getDriver(void)
{
	return *m_pDriver;
}

uint32 CAcquisitionServerGUI::getSampleCountPerBuffer(void)
{
	return m_ui32SampleCountPerBuffer;
}

uint32 CAcquisitionServerGUI::getTCPPort(void)
{
	return ::gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(::gtk_builder_get_object(m_pBuilderInterface, "spinbutton_connection_port")));
}

IHeader& CAcquisitionServerGUI::getHeaderCopy(void)
{
	return m_oHeaderCopy;
}

void CAcquisitionServerGUI::setClientText(const char *sClientText)
{
	gtk_label_set_label(GTK_LABEL(gtk_builder_get_object(m_pBuilderInterface, "label_connected_host_count")), sClientText);
}

void CAcquisitionServerGUI::setStateText(const char *sStateText)
{
	gtk_label_set_label(GTK_LABEL(gtk_builder_get_object(m_pBuilderInterface, "label_status")), sStateText);
}

void CAcquisitionServerGUI::setDriftMs(float64 f64DriftMs)
{
	const uint64 l_ui64DriftToleranceDurationMs = m_pAcquisitionServer->m_oDriftCorrection.getDriftToleranceDurationMs();
	float64 l_f64DriftRatio = f64DriftMs / static_cast<float64>(l_ui64DriftToleranceDurationMs);
	boolean l_bDriftWarning=false;
	char l_sLabel[1024];

#ifdef TIMINGDEBUG
	std::cout << "GUI drift " << f64Drift << " rat " << l_f64DriftRatio << "\n";
#endif

	if(l_f64DriftRatio<-1)
	{
		l_f64DriftRatio=-1;
		l_bDriftWarning=true;
	}

	if(l_f64DriftRatio>1)
	{
		l_f64DriftRatio=1;
		l_bDriftWarning=true;
	}

	if(l_f64DriftRatio<0)
	{
		gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(gtk_builder_get_object(m_pBuilderInterface, "progressbar_drift_1")), -l_f64DriftRatio);
		gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(gtk_builder_get_object(m_pBuilderInterface, "progressbar_drift_2")), 0);
	}
	else
	{
		gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(gtk_builder_get_object(m_pBuilderInterface, "progressbar_drift_1")), 0);
		gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(gtk_builder_get_object(m_pBuilderInterface, "progressbar_drift_2")), l_f64DriftRatio);
	}

	if(l_bDriftWarning)
	{
		::sprintf(l_sLabel, "<b>Device drift is too high</b> : %3.2lf ms\n<small>late &lt;-- (tolerance is set to %u ms) --&gt; early</small>", f64DriftMs, static_cast<unsigned int>(l_ui64DriftToleranceDurationMs));
	}
	else
	{
		::sprintf(l_sLabel, "Device drift : %3.2lf ms\n<small>late &lt;-- (tolerance is set to %u ms) --&gt; early</small>", f64DriftMs, static_cast<unsigned int>(l_ui64DriftToleranceDurationMs));
	}
	::gtk_label_set_markup(GTK_LABEL(gtk_builder_get_object(m_pBuilderInterface, "label_drift")), l_sLabel);
}

void CAcquisitionServerGUI::setImpedance(OpenViBE::uint32 ui32ChannelIndex, OpenViBE::float64 f64Impedance)
{
	if(m_pImpedanceWindow)
	{
		if(f64Impedance>=0)
		{
			//float64 l_dFraction=(f64Impedance*.001/20); With fixed impedance limit, 20kOhm max / 25%=5kOhm to be good
			float64 l_dFraction=(f64Impedance / (m_rKernelContext.getConfigurationManager().expandAsFloat("${AcquisitionServer_DefaultImpedanceLimit}",5000) * 4));
			if(l_dFraction>1) l_dFraction=1;

			char l_sMessage[1024];
			char l_sLabel[1024];
			char l_sImpedance[1024];
			char l_sStatus[1024];

			if(::strcmp(m_oHeaderCopy.getChannelName(ui32ChannelIndex), ""))
			{
				::strcpy(l_sLabel, m_oHeaderCopy.getChannelName(ui32ChannelIndex));
			}
			else
			{
				::sprintf(l_sLabel, "Channel %i", ui32ChannelIndex+1);
			}

			if(l_dFraction==1)
			{
				::sprintf(l_sImpedance, "Too high !");
			}
			else
			{
				::sprintf(l_sImpedance, "%.2f kOhm", f64Impedance*.001);
			}

			::sprintf(l_sStatus, "%s", l_dFraction<.25?"Good !":"Bad...");
			::sprintf(l_sMessage, "%s\n%s\n\n%s", l_sLabel, l_sImpedance, l_sStatus);

			::gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), l_dFraction);
			::gtk_progress_bar_set_text(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), l_sMessage);
		}
		else
		{
			::gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), 0);
			if(f64Impedance==OVAS_Impedance_Unknown)
			{
				::gtk_progress_bar_set_text(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), "Measuring...");
			}
			else if (f64Impedance==OVAS_Impedance_NotAvailable)
			{
				::gtk_progress_bar_set_text(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), "n/a");
			}
			else
			{
				::gtk_progress_bar_set_text(GTK_PROGRESS_BAR(m_vLevelMesure[ui32ChannelIndex]), "Unknown");
			}
		}
	}
}

void CAcquisitionServerGUI::disconnect(void)
{
	::GtkToggleButton* l_pConnectButton=GTK_TOGGLE_BUTTON(gtk_builder_get_object(m_pBuilderInterface, "togglebutton_connect"));

	if(gtk_toggle_button_get_active(l_pConnectButton))
	{
		gtk_toggle_button_set_active(l_pConnectButton, false);
	}
}

//___________________________________________________________________//
//                                                                   //

void CAcquisitionServerGUI::buttonConnectToggledCB(::GtkToggleButton* pButton)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "buttonConnectToggledCB\n";

	if(gtk_toggle_button_get_active(pButton))
	{
		if(m_ui32SampleCountPerBuffer!=uint32(-1) && m_pAcquisitionServerThread->connect())
		{
			// Impedance window creation
			{
				const uint64 l_ui64ColumnCount=m_rKernelContext.getConfigurationManager().expandAsInteger("${AcquisitionServer_CheckImpedance_ColumnCount}", 8);
				const uint32 l_ui32LineCount=(uint32)(m_oHeaderCopy.getChannelCount()/l_ui64ColumnCount);
				const uint32 l_ui32LastCount=(uint32)(m_oHeaderCopy.getChannelCount()%l_ui64ColumnCount);

				::GtkWidget* l_pTable=gtk_table_new((gint)(l_ui32LineCount+(l_ui32LastCount?1:0)), (gint)((l_ui32LineCount?l_ui64ColumnCount:l_ui32LastCount)), true);

				for(uint32 i=0; i<m_oHeaderCopy.getChannelCount(); i++)
				{
					const uint32 j=(uint32)(i/l_ui64ColumnCount);
					const uint32 k=(uint32)(i%l_ui64ColumnCount);
					::GtkWidget* l_pProgressBar=::gtk_progress_bar_new();
					::gtk_progress_bar_set_orientation(GTK_PROGRESS_BAR(l_pProgressBar), GTK_PROGRESS_BOTTOM_TO_TOP);
					::gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(l_pProgressBar), 0);
					::gtk_progress_bar_set_text(GTK_PROGRESS_BAR(l_pProgressBar), "n/a");
					::gtk_table_attach_defaults(GTK_TABLE(l_pTable), l_pProgressBar, k, k+1, j, j+1);
					m_vLevelMesure.push_back(l_pProgressBar);
				}

				m_pImpedanceWindow=::gtk_window_new(GTK_WINDOW_TOPLEVEL);
				::gtk_window_set_title(GTK_WINDOW(m_pImpedanceWindow), "Impedance check");
				::gtk_container_add(GTK_CONTAINER(m_pImpedanceWindow), l_pTable);
				if(m_pAcquisitionServer->isImpedanceCheckRequested())
				{
					::gtk_widget_show_all(m_pImpedanceWindow);
				}
			}

			gtk_button_set_label(GTK_BUTTON(pButton), "gtk-disconnect");

			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_configure")), false);
			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_preference")), false);
			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_play")), true);
			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_stop")), false);

			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "spinbutton_connection_port")), false);
			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "combobox_sample_count_per_sent_block")), false);
			gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "combobox_driver")), false);
		}
		else
		{
			if(m_ui32SampleCountPerBuffer==uint32(-1))
			{
				m_rKernelContext.getLogManager() << LogLevel_Warning << "Sample count per sent block is invalid.\n";
			}

			gtk_toggle_button_set_active(pButton, false);
		}
	}
	else
	{
		m_pAcquisitionServerThread->disconnect();

		if(m_pImpedanceWindow)
		{
			::gtk_widget_destroy(m_pImpedanceWindow);
			m_vLevelMesure.clear();
			m_pImpedanceWindow=NULL;
		}

		gtk_button_set_label(GTK_BUTTON(pButton), "gtk-connect");

		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_configure")), m_pDriver->isConfigurable());
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_preference")), true);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_play")), false);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_stop")), false);

		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "spinbutton_connection_port")), true);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "combobox_sample_count_per_sent_block")), true);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "combobox_driver")), true);

		setClientText("");
	}
}

void CAcquisitionServerGUI::buttonStartPressedCB(::GtkButton* pButton)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "buttonStartPressedCB\n";

	if(m_pAcquisitionServerThread->start())
	{
		if(m_pImpedanceWindow)
		{
			::gtk_widget_hide(m_pImpedanceWindow);
		}

		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_play")), false);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_stop")), true);

		setStateText("Starting...");
	}
	else
	{
		setStateText("Start failed !");
		setClientText("");
	}
}

void CAcquisitionServerGUI::buttonStopPressedCB(::GtkButton* pButton)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "buttonStopPressedCB\n";

	if(m_pAcquisitionServerThread->stop())
	{
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_play")), true);
		gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_stop")), false);
	}
	else
	{
		setStateText("Stop failed !");
		setClientText("");
	}
}

void CAcquisitionServerGUI::buttonPreferencePressedCB(::GtkButton* pButton)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "buttonPreferencePressedCB\n";

	::GtkBuilder* l_pInterface=gtk_builder_new();
	::gtk_builder_add_from_file(l_pInterface, OVAS_GUI_File, NULL);
	::GtkDialog* l_pDialog=GTK_DIALOG(::gtk_builder_get_object(l_pInterface, "openvibe-acquisition-server-configuration"));
	::GtkComboBox* l_pDriftCorrectionPolicy=GTK_COMBO_BOX(::gtk_builder_get_object(l_pInterface, "combobox_drift_correction"));
	::GtkComboBox* l_pNaNReplacementPolicy=GTK_COMBO_BOX(::gtk_builder_get_object(l_pInterface, "combobox_nan_replacement"));
	::GtkSpinButton* l_pDriftTolerance=GTK_SPIN_BUTTON(::gtk_builder_get_object(l_pInterface, "spinbutton_drift_tolerance"));
	::GtkSpinButton* l_pJitterMeasureCount=GTK_SPIN_BUTTON(::gtk_builder_get_object(l_pInterface, "spinbutton_jitter_measure_count"));
	::GtkSpinButton* l_pOverSamplingFactor=GTK_SPIN_BUTTON(::gtk_builder_get_object(l_pInterface, "spinbutton_oversampling_factor"));
	::GtkToggleButton* l_ChannelSelection=GTK_TOGGLE_BUTTON(::gtk_builder_get_object(l_pInterface, "checkbutton_channel_selection"));

	::gtk_combo_box_set_active(l_pDriftCorrectionPolicy, (int)m_pAcquisitionServer->m_oDriftCorrection.getDriftCorrectionPolicy());
	::gtk_spin_button_set_value(l_pDriftTolerance, (gdouble)m_pAcquisitionServer->m_oDriftCorrection.getDriftToleranceDurationMs());
	::gtk_spin_button_set_value(l_pJitterMeasureCount, (gdouble)m_pAcquisitionServer->m_oDriftCorrection.getJitterEstimationCountForDrift());
	::gtk_spin_button_set_value(l_pOverSamplingFactor, (gdouble)m_pAcquisitionServer->getOversamplingFactor());
	::gtk_toggle_button_set_active(l_ChannelSelection, m_pAcquisitionServer->isChannelSelectionRequested()?TRUE:FALSE);
	::gtk_combo_box_set_active(l_pNaNReplacementPolicy, (int)m_pAcquisitionServer->getNaNReplacementPolicy());

	// Load the settings for the plugins

	::GtkTable* l_pSettingsTable = GTK_TABLE(::gtk_builder_get_object(l_pInterface, "table-pluginsettings"));

	gtk_table_resize(l_pSettingsTable, m_vPluginProperties.size(), 2);

	for (size_t setting_index = 0; setting_index < m_vPluginProperties.size(); ++setting_index)
	{
		Property* l_pCurrentProperty = m_vPluginProperties[setting_index].m_pProperty;

		// Create the setting controller widget
		::GtkWidget* l_pSettingControl = NULL;

		if ( const TypedProperty<boolean>* r = dynamic_cast< const TypedProperty<boolean>* >(l_pCurrentProperty))
		{
			// cout << "bool\n";
			l_pSettingControl = gtk_check_button_new();
			gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(l_pSettingControl), *(r->getData()));
		}
		else if( const TypedProperty<CString>* r = dynamic_cast< const TypedProperty<CString>* >(l_pCurrentProperty))
		{
			// cout << "string\n";
			l_pSettingControl = gtk_entry_new();
			gtk_entry_append_text(GTK_ENTRY(l_pSettingControl), r->getData()->toASCIIString());
		} 
		else if( const TypedProperty<uint32>* r = dynamic_cast< const TypedProperty<uint32>* >(l_pCurrentProperty)) 
		{
			// cout << "uinteger\n";
			l_pSettingControl = gtk_spin_button_new_with_range((gdouble)std::numeric_limits<uint32>::min(), (gdouble)std::numeric_limits<uint32>::max(), 1.0);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(l_pSettingControl), (gdouble)*(r->getData()));
		} 
		else if( const TypedProperty<int64>* r = dynamic_cast< const TypedProperty<int64>* >(l_pCurrentProperty)) 
		{
			// cout << "integer\n";
			l_pSettingControl = gtk_spin_button_new_with_range((gdouble)std::numeric_limits<int64>::min(), (gdouble)std::numeric_limits<int64>::max(), 1.0);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(l_pSettingControl), (gdouble)*(r->getData()));
		} 
		else if( const TypedProperty<float32>* r = dynamic_cast< const TypedProperty<float32>* >(l_pCurrentProperty)) 
		{
			// cout << "float32\n";
			l_pSettingControl = gtk_spin_button_new_with_range((gdouble)std::numeric_limits<float32>::min(), (gdouble)std::numeric_limits<float32>::max(), 1.0);
			gtk_spin_button_set_digits(GTK_SPIN_BUTTON(l_pSettingControl), 5);
			gtk_spin_button_set_increments(GTK_SPIN_BUTTON(l_pSettingControl), 0.1f, 1.0f);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(l_pSettingControl), (gdouble)*(r->getData()));
		} 
		else 
		{
			// cout << "unknown\n";
			l_pSettingControl = gtk_label_new("Undefined Type");
		}

		if(l_pSettingControl) {
			// Create label
			::GtkWidget* l_pSettingLabel = gtk_label_new( l_pCurrentProperty->getName().toASCIIString() );

			// align to left
			gtk_misc_set_alignment( GTK_MISC(l_pSettingLabel), 0.0, 0.0 );

			// insert the settings into the table
			gtk_table_attach(l_pSettingsTable, l_pSettingLabel,   0, 1, setting_index, setting_index+1, GTK_FILL, GTK_SHRINK,   2, 0);
			gtk_table_attach_defaults(l_pSettingsTable, l_pSettingControl, 1, 2, setting_index, setting_index+1);

			m_vPluginProperties[setting_index].m_pWidget = l_pSettingControl;
			gtk_widget_show(l_pSettingLabel);
			gtk_widget_show(l_pSettingControl);
		}
	}

	gint l_iResponseId=::gtk_dialog_run(l_pDialog);
	switch(l_iResponseId)
	{
		case GTK_RESPONSE_APPLY:
		case GTK_RESPONSE_OK:
		case GTK_RESPONSE_YES:
			m_pAcquisitionServer->setNaNReplacementPolicy((ENaNReplacementPolicy)::gtk_combo_box_get_active(l_pNaNReplacementPolicy));
			m_pAcquisitionServer->m_oDriftCorrection.setDriftCorrectionPolicy((EDriftCorrectionPolicy)::gtk_combo_box_get_active(l_pDriftCorrectionPolicy));
			m_pAcquisitionServer->m_oDriftCorrection.setDriftToleranceDurationMs(::gtk_spin_button_get_value_as_int(l_pDriftTolerance));
			m_pAcquisitionServer->m_oDriftCorrection.setJitterEstimationCountForDrift(::gtk_spin_button_get_value_as_int(l_pJitterMeasureCount));
			m_pAcquisitionServer->setOversamplingFactor(::gtk_spin_button_get_value_as_int(l_pOverSamplingFactor));
			m_pAcquisitionServer->setChannelSelectionRequest(::gtk_toggle_button_get_active(l_ChannelSelection)?true:false);

			// Side-effect: Update the tolerance ms
			setDriftMs(0);

			for (size_t setting_index = 0; setting_index < m_vPluginProperties.size(); ++setting_index)
			{
				Property* l_pCurrentProperty = m_vPluginProperties[setting_index].m_pProperty;

				if ( TypedProperty<boolean>* r = dynamic_cast< TypedProperty<boolean>* >(l_pCurrentProperty))
				{
					// cout << "bool\n";
					boolean tmp = ::gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(m_vPluginProperties[setting_index].m_pWidget)) ? true : false;
					r->replaceData(tmp);
				}
				else if( TypedProperty<CString>* r = dynamic_cast< TypedProperty<CString>* >(l_pCurrentProperty))
				{
					CString tmp = CString(::gtk_entry_get_text(GTK_ENTRY(m_vPluginProperties[setting_index].m_pWidget)));
					// cout << "string: " << tmp.toASCIIString() << "\n";
					r->replaceData( tmp );
				}
				else if( TypedProperty<int64>* r = dynamic_cast< TypedProperty<int64>* >(l_pCurrentProperty))
				{
					// cout << "integer\n";
					int64 tmp = static_cast<int64>(::gtk_spin_button_get_value(GTK_SPIN_BUTTON(m_vPluginProperties[setting_index].m_pWidget)));
					r->replaceData( tmp );
				} 
				else if( TypedProperty<uint32>* r = dynamic_cast< TypedProperty<uint32>* >(l_pCurrentProperty))
				{
					// cout << "uinteger\n";
					uint32 tmp = static_cast<uint32>(::gtk_spin_button_get_value(GTK_SPIN_BUTTON(m_vPluginProperties[setting_index].m_pWidget)));
					r->replaceData( tmp );
				} 
				else if( TypedProperty<float32>* r = dynamic_cast< TypedProperty<float32>* >(l_pCurrentProperty)) 
				{
					// cout << "float32\n";
					float32 tmp = static_cast<float32>(::gtk_spin_button_get_value(GTK_SPIN_BUTTON(m_vPluginProperties[setting_index].m_pWidget)));
					r->replaceData( tmp );
				} 
				else 
				{
					// cout << "unknown\n";
				}
			}


			break;
		case GTK_RESPONSE_CANCEL:
		case GTK_RESPONSE_NO:
			break;
	}

	::gtk_widget_destroy(GTK_WIDGET(l_pDialog));
	g_object_unref(l_pInterface);
}

void CAcquisitionServerGUI::buttonConfigurePressedCB(::GtkButton* pButton)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "buttonConfigurePressedCB\n";

	if(m_pDriver->isConfigurable())
	{
		m_pDriver->configure();
	}
}

void CAcquisitionServerGUI::comboBoxDriverChanged(::GtkComboBox* pComboBox)
{
	m_rKernelContext.getLogManager() << LogLevel_Debug << "comboBoxDriverChanged\n";
	m_pDriver=m_vDriver[gtk_combo_box_get_active(pComboBox)];
	gtk_widget_set_sensitive(GTK_WIDGET(gtk_builder_get_object(m_pBuilderInterface, "button_configure")), m_pDriver->isConfigurable());
}

void CAcquisitionServerGUI::comboBoxSampleCountPerSentBlockChanged(::GtkComboBox* pComboBox)
{
	int l_iSampleCountPerSentBlock=0;
	m_rKernelContext.getLogManager() << LogLevel_Debug << "comboBoxSampleCountPerSentBlockChanged\n";
	if(::sscanf(::gtk_combo_box_get_active_text(GTK_COMBO_BOX(::gtk_builder_get_object(m_pBuilderInterface, "combobox_sample_count_per_sent_block"))), "%i", &l_iSampleCountPerSentBlock)==1 && l_iSampleCountPerSentBlock>0)
	{
		m_ui32SampleCountPerBuffer=uint32(l_iSampleCountPerSentBlock);
	}
	else
	{
		m_ui32SampleCountPerBuffer=uint32(-1);
	}
}

void CAcquisitionServerGUI::registerPlugin(IAcquisitionServerPlugin* plugin)
{
	if (m_pAcquisitionServer != NULL)
	{
		m_pAcquisitionServer->m_vPlugins.push_back(plugin);
	}
}

/**
  * \brief This function scans all registered plugins for settings.
  *
  * All of the plugins are inserted into a vector containing the pointer to the actual settings structure
  * along with a unique name for settings.
  */
void CAcquisitionServerGUI::scanPluginSettings()
{
	vector<IAcquisitionServerPlugin*> l_vPlugins = m_pAcquisitionServer->getPlugins();

	m_vPluginProperties.clear();

	for(auto itp = l_vPlugins.begin(); itp != l_vPlugins.end(); ++itp)
	{
		IAcquisitionServerPlugin* l_pPlugin = dynamic_cast<IAcquisitionServerPlugin*>(*itp);

		SettingsHelper& tmp = l_pPlugin->getSettingsHelper();
		const std::map<OpenViBE::CString, Property*>& props = tmp.getAllProperties();

		for(auto prop_it = props.begin();prop_it!=props.end();++prop_it) {
			m_vPluginProperties.push_back( PropertyAndWidget(prop_it->second, NULL) );
		}

	}

}

void CAcquisitionServerGUI::savePluginSettings()
{
	vector<IAcquisitionServerPlugin*> l_vPlugins = m_pAcquisitionServer->getPlugins();

	for(auto itp = l_vPlugins.begin(); itp != l_vPlugins.end(); ++itp)
	{
		IAcquisitionServerPlugin* l_pPlugin = dynamic_cast<IAcquisitionServerPlugin*>(*itp);

		SettingsHelper& tmp = l_pPlugin->getSettingsHelper();
		tmp.save();
	}
}
