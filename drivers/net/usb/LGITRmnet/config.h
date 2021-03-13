#define DRIVER_VERSION		"1.3.4.0"
#define CHAR_DEVICE_NAME	"qcqmi"
#define NET_DEVICE_NAME		"rmnet"
#define NET_MODULE_DESC		"USB Network Driver for LG Innotech Network Port"
#define NET_INTERFACE_NUM	7
#define DEVICE_MATCH_INFO	.match_flags		= USB_DEVICE_ID_MATCH_DEVICE|\
												  USB_DEVICE_ID_MATCH_INT_INFO,\
							.idVendor			= 0x1d74,\
							.idProduct			= 0x3100,\
							.bInterfaceClass	= 0xFF,\
							.bInterfaceSubClass	= 0x10,\
							.bInterfaceProtocol	= 0xFF

#define QMI_PRESET_QMAP

#ifdef QMI_PRESET_QMAP
	#define QOS_DATA_FORMAT 0						// QOS flow header is not present
	#define LINK_LAYER_PROTOCOL 0x02				// IP mode
	#define UL_DATA_AGGREGATION_PROT 0x05			// UL QMAP is enabled
	#define DL_DATA_AGGREGATION_PROT 0x05			// DL QMAP is enabled
	#define DL_DATA_AGGREGATION_MAX_DATAGRAMS 0x10	// Maximum number of datagrams
	#define DL_DATA_AGGREGATION_MAX_SIZE 0x1000		// Maximum single aggregated packet size
	#define UL_DATA_AGGREGATION_MAX_DATAGRAMS 0x10
	#define UL_DATA_AGGREGATION_MAX_SIZE 0x1000
	#define PERIPHERAL_EP_ID_EP_TYPE 0x02			// High-speed universal serial bus
	#define PERIPHERAL_EP_ID_IFACE_ID NET_INTERFACE_NUM
	#define QMAP_IN_BAND_FLOW_CONTROL 0x01			// Flow control will be done by the TE
	#define QMAP_MUX_ID 0x81
	#define QMAP_CLIENT_TYPE 0x1					// Client type is tethered
#else
	#define QOS_DATA_FORMAT 0						// QOS flow header is not present
	#define LINK_LAYER_PROTOCOL 0x02				// IP mode
	#define UL_DATA_AGGREGATION_PROT 0x00			// UL data aggregation is disabled
	#define DL_DATA_AGGREGATION_PROT 0x00			// DL data aggregation is disabled
	#define PERIPHERAL_EP_ID_EP_TYPE 0x02			// High-speed universal serial bus
	#define PERIPHERAL_EP_ID_IFACE_ID NET_INTERFACE_NUM
	#define QMAP_MUX_ID 0x81
	#define QMAP_CLIENT_TYPE 0x1					// Client type is tethered
#endif
