#ifndef _XBEE_DRIVER_H_
#define _XBEE_DRIVER_H_

// ====================================================
//               INTERNAL DRIVER ERROR CODE
// ====================================================
typedef enum {
    XBEE_OK = 0,
    XBEE_TIMEOUT,
    XBEE_CHECKSUM_ERROR,
    XBEE_WRITE_ERROR,
    XBEE_INIT_ERROR,
}tXbeeErr;

// ====================================================
//               XBEE SETTINGS
// ====================================================
typedef struct {
    char PANID[4];          // XBEE NETWORK ID      (4 char : 0 à F en ASCII)
    char CHANNEL[2];        // Channel du reseau    (2 char : 0 à F en ASCII)
    unsigned char ID;             //XBEE ID               (4 char : 0 à F en ASCII)
    unsigned char APIMODE;           //Definit le mode de transmission (0 ou 1 en ASCII)
    unsigned char SECURITY;          //Active la clé reseau  (0 ou 1 en ASCII)
    char KEY[32];            //clé reseau            (32 char : 0 à F en ASCII)
    unsigned char COORDINATOR;       //Coordinateur du reseau ou routeur (0 ou 1 en ASCII)
    unsigned char COORDINATOR_OPTION;      //Option du coordinateur (0x04 = autorise les Xbee à rejoindre son reseau)
}tXbeeSettings;


// ====================================================
//               XBEE RAW MESSAGE
// ====================================================
#define XBEE_MAX_DATA_LEN    80
typedef struct {
    unsigned char FrameType;        //Type de message
    unsigned int SourceID;          //Source du message envoyé
    unsigned short DestinationID;   //! TODO : peut Ãªtre qu'on en a pas besoin Ã  ce niveau lÃ  ?? A supprimer sinon
    unsigned int DLC; // Data Lenght
    unsigned char Data[XBEE_MAX_DATA_LEN];
    unsigned char Checksum;
}tXbeeMessage;

// ====================================================
//               XBEE RECEPTION STATE
// ====================================================
typedef enum {
    XBEE_HEADER = 0,
    XBEE_LENGTH_MSB,
    XBEE_LENGTH_LSB,
    XBEE_FRAME,
    XBEE_SOURCE_MSB,
    XBEE_SOURCE_LSB,
    XBEE_RSSI,
    XBEE_OPTION,
    XBEE_DATA,
    XBEE_CHECKSUM,
}xmessage_state;

// ====================================================
//
// ====================================================
#define XBEE_BROADCAST_ID 0xFFFF  //valeur de la doc XBEE qui permet d'adresser tous les XBEE

// ====================================================
//        BASE CLASS FOR XBEE DRIVER
// ====================================================
    // call this method to inform driver a new data was received on serial port
     void xbee_decode(unsigned char newData);
     void xbee_decode_buff(unsigned char *buff_data, unsigned char buff_size);

    // call this method to encode and send data in a Xbee format packet
     void xbee_encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address);

    //call this method to initialize Xbee register
     void xbee_setRegister_buffer(char *parameter, unsigned char *value, unsigned short valueLength);
     void xbee_setRegister_byte(char *parameter, unsigned char value);

    //call this method to get the value of a register or apply the parameters
     void xbee_getRegister(char *parameter);

    //call this method to check the setting of a register is ok
     tXbeeErr xbee_decodeInit(unsigned char *buff_data, unsigned char buff_size);

    // High level API
     tXbeeErr xbee_init(const tXbeeSettings* settings);
     tXbeeErr xbee_connect();
     unsigned char xbee_isPresent(unsigned char xbee_id);
     unsigned char xbee_isConnected();
     tXbeeErr xbee_reset();

    void xbee_setID(unsigned short id);
    unsigned char xbee_getID();


    //! Compute checksum from the packet
    unsigned char xbee_getChecksum(unsigned char *xpacket);


#endif // _XBEE_DRIVER_H_
