#include "USBconstant.h"


//Device descriptor
const uint8_t DevDesc[] = {
    0x12,0x01,
    0x10,0x01,  //USB spec release number in BCD format, USB1.1 (0x10, 0x01).
    0xEF,0x02,0x01, //bDeviceClass, bDeviceSubClass, bDeviceProtocol 
    DEFAULT_ENDP0_SIZE, //bNumConfigurations
    0x09,0x12,0x5C,0xC5, // VID PID 
    0x01,0x01,  //version
    0x01,0x02,0x03, //bString
    0x01    //bNumConfigurations
};

const uint16_t DevDescLen = sizeof(DevDesc);

#define CfgDesc_SIZE_For_GCC 100

const uint8_t CfgDesc[] ={
    0x09,0x02,CfgDesc_SIZE_For_GCC & 0xff,CfgDesc_SIZE_For_GCC >> 8,
    0x03,0x01,0x00,0x80,0x64,             //Configuration descriptor (3 interfaces)
    // Interface Association Descriptor, IAD, this packes following 2 interfaces into 1
    0x08,0x0B,0x00,0x02,0x02,0x02,0x01,0x04,
    // Interface 1 (CDC) descriptor
    0x09,0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x04,    // CDC control description, 1 endpoint
    // Functional Descriptor refer to usbcdc11.pdf
    0x05,0x24,0x00,0x10,0x01,                                 //Header Functional Descriptor
    0x05,0x24,0x01,0x00,0x00,                                 //Call Management Functional Descriptor
    0x04,0x24,0x02,0x02,                                      //Direct Line Management Functional Descriptor, Support: Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, Serial_State 
    0x05,0x24,0x06,0x00,0x01,                                 //Union Functional Descriptor, Communication class interface 0, Data Class Interface 1
    0x07,0x05,0x81,0x03,0x08,0x00,0x40,                       //EndPoint descriptor (CDC Upload, Interrupt)
    // Interface 2 (Data Interface) descriptor
    0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x04,             //Data Class Interface descriptor
    0x07,0x05,0x02,0x02,0x40,0x00,0x00,                       //endpoint descriptor
    0x07,0x05,0x82,0x02,0x40,0x00,0x00,                       //endpoint descriptor
    // Interface 3 (HID Keyboard) descriptor
    0x09,0x04,0x02,0x00,0x01,0x03,0x01,0x01,0x00,             //HID Keyboard Interface descriptor
    0x09,0x21,0x10,0x01,0x00,0x01,0x22,0x7c,0x00,             //HID Descriptor
    0x07,0x05,0x83,0x03,0x09,0x00,0x0A,                       //EndPoint descriptor (HID Keyboard Upload, Interrupt)
};

const uint16_t CfgDescLen = sizeof(CfgDesc);

//String Descriptors
const uint8_t LangDes[]={0x04,0x03,0x09,0x04};           //Language Descriptor
const uint16_t LangDesLen = sizeof(LangDes);
const uint16_t SerDes[]={                                 //Serial String Descriptor
#if defined(CH573)
    0x030C,
    'C','H','5','7','x',
#elif defined(CH32X035)
    0x0312,
    'C','H','3','2','X','0','3','5',
#endif
};
const uint16_t SerDesLen = sizeof(SerDes);
const uint16_t Prod_Des[]={                                //Produce String Descriptor
#if defined(CH573)
    0x0316,
    'C','H','5','7','x','d','u','i','n','o',
#elif defined(CH32X035)
    0x031C,
    'C','H','3','2','X','0','3','5','d','u','i','n','o',
#endif
};
const uint16_t Prod_DesLen = sizeof(Prod_Des);

const uint16_t CDC_Des[]={
    0x0316,
    'C','D','C',' ','S','e','r','i','a','l',
};
const uint16_t CDC_DesLen = sizeof(CDC_Des);

const uint16_t Manuf_Des[]={    //SDCC is little endian
    0x030E,
    'D','e','q','i','n','g',
};
const uint16_t Manuf_DesLen = sizeof(Manuf_Des);

const uint8_t ReportDescriptor[] = {
    0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,       // USAGE (Keyboard)
    0xa1, 0x01,       // COLLECTION (Application)
    0x85, 0x01,       //   REPORT_ID (1)
    0x05, 0x07,       //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,       //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,       //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,       //   LOGICAL_MINIMUM (0)
    0x25, 0x01,       //   LOGICAL_MAXIMUM (1)
    0x95, 0x08,       //   REPORT_COUNT (8)
    0x75, 0x01,       //   REPORT_SIZE (1)
    0x81, 0x02,       //   INPUT (Data,Var,Abs)
    0x95, 0x01,       //   REPORT_COUNT (1)
    0x75, 0x08,       //   REPORT_SIZE (8)
    0x81, 0x03,       //   INPUT (Cnst,Var,Abs)
    0x95, 0x06,       //   REPORT_COUNT (6)
    0x75, 0x08,       //   REPORT_SIZE (8)
    0x15, 0x00,       //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00, //   LOGICAL_MAXIMUM (255)
    0x05, 0x07,       //   USAGE_PAGE (Keyboard)
    0x19, 0x00,       //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0xe7,       //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x81, 0x00,       //   INPUT (Data,Ary,Abs)
    0x05, 0x08,       //   USAGE_PAGE (LEDs)
    0x19, 0x01,       //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,       //   USAGE_MAXIMUM (Kana)
    0x15, 0x00,       //   LOGICAL_MINIMUM (0)
    0x25, 0x01,       //   LOGICAL_MAXIMUM (1)
    0x95, 0x05,       //   REPORT_COUNT (5)
    0x75, 0x01,       //   REPORT_SIZE (1)
    0x91, 0x02,       //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,       //   REPORT_COUNT (1)
    0x75, 0x03,       //   REPORT_SIZE (3)
    0x91, 0x03,       //   OUTPUT (Cnst,Var,Abs)
    0xc0,             // END_COLLECTION
    0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,       // USAGE (Mouse)
    0xa1, 0x01,       // COLLECTION (Application)
    0x09, 0x01,       //   USAGE (Pointer)
    0xa1, 0x00,       //   COLLECTION (Physical)
    0x85, 0x02,       //   REPORT_ID (2)
    0x05, 0x09,       //     USAGE_PAGE (Button)
    0x19, 0x01,       //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,       //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,       //     LOGICAL_MINIMUM (0)
    0x25, 0x01,       //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,       //     REPORT_COUNT (3)
    0x75, 0x01,       //     REPORT_SIZE (1)
    0x81, 0x02,       //     INPUT (Data,Var,Abs)
    0x95, 0x01,       //     REPORT_COUNT (1)
    0x75, 0x05,       //     REPORT_SIZE (5)
    0x81, 0x03,       //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,       //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,       //     USAGE (X)
    0x09, 0x31,       //     USAGE (Y)
    0x09, 0x38,       //     USAGE (Wheel)
    0x15, 0x81,       //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,       //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,       //     REPORT_SIZE (8)
    0x95, 0x03,       //     REPORT_COUNT (3)
    0x81, 0x06,       //     INPUT (Data,Var,Rel)
    0xc0,             //     END_COLLECTION
    0xc0              // END_COLLECTION
};

const uint16_t ReportDescriptor_Len = sizeof(ReportDescriptor);
