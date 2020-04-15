#include <stdio.h>
#include <string.h>
#include "xbeedriver.h"

//Parametres xbee
static unsigned char m_panid[4];
static unsigned char m_channel[2];
static unsigned char m_id = 0;
static unsigned char m_apimode;
static unsigned char m_security;
static unsigned char m_key[32];
static unsigned char m_coordinator;
static unsigned char m_coordinator_option;

//! Current packet
static tXbeeMessage m_current_xmessage;   //"xmessage" = Xbee message
static unsigned char m_xmessage_index = 0;
static xmessage_state m_xmessage_state = XBEE_HEADER;

// External functions (to be re-implemented)
// this method is called by driver to inform a valid buffer ws receinved and now ready to be used by application
extern void xbee_readyBytes_callback(unsigned char *buff_data, unsigned char buff_size, unsigned short source_id);
// this method is called by driver to write a buffer to physical serial port on specific hardware
extern void xbee_write(unsigned char *buff_data, unsigned char buff_size);
// this method is called by driver to request a delay on specific hardware
extern void xbee_delay_us(unsigned long delay);


// ____________________________________________________________
unsigned char xbee_getID()
{
    return m_id;
}

// ____________________________________________________________
void xbee_setID(unsigned short id)
{
    m_id = id;
}

// ____________________________________________________________
unsigned char xbee_isPresent(unsigned char id)
{
    unsigned char dummy = 1;
    return (dummy);
}

// ____________________________________________________________
tXbeeErr xbee_init(const tXbeeSettings *settings)
{
    tXbeeErr dummy=XBEE_OK;
    int i;
    m_id = settings->ID;
    for (i=0; i< 4; i++){
        m_panid[i] = settings->PANID[i];
    }
    for (i=0; i< 2; i++){
        m_channel[i] = settings->CHANNEL[i];
    }
    m_apimode = settings->APIMODE;
    m_security = settings->SECURITY;
    for (i=0; i< 32; i++){
        m_key[i] = settings->KEY[i];
    }
    m_coordinator = settings->COORDINATOR;
    m_coordinator_option = settings->COORDINATOR_OPTION;

    //! Mettre un delay d'une sec
    unsigned long delay = 100000;
    unsigned long delay_1sec2 = 1200000;

    xbee_delay_us(delay_1sec2);
    unsigned char Plus[] = "+++";
    xbee_write(Plus, 3);
    xbee_delay_us(delay_1sec2);

    xbee_setRegister_buffer("ID", m_panid, 4);
    xbee_delay_us(delay);
    xbee_setRegister_buffer("CH", m_channel, 2);
    xbee_delay_us(delay);
    xbee_setRegister_byte("MY", m_id);
    xbee_delay_us(delay);
    xbee_setRegister_byte("CE", m_coordinator);
    xbee_delay_us(delay);
    xbee_setRegister_byte("EE", m_security);
    xbee_delay_us(delay);
    xbee_setRegister_buffer("KY", m_key, 32);
    xbee_delay_us(delay);
    xbee_setRegister_byte("A2", m_coordinator_option);
    xbee_delay_us(delay);
    xbee_setRegister_byte("AP", m_apimode);
    xbee_delay_us(delay);
    xbee_getRegister("CN");      //apply parameters
    xbee_delay_us(delay_1sec2);

    m_xmessage_state = XBEE_HEADER;

    return dummy;
}


// ____________________________________________________________
void xbee_setRegister_buffer(char* parameter, unsigned char *value, unsigned short valueLength)
{
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = param_size + valueLength + 3;
    unsigned char data[dataLength];
    int i;
    data[0] = 'A';
    data[1] = 'T';
    for (i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    for (i=0; i<valueLength; i++){
        data[i + param_size + 2] = value[i];
    }
    data[dataLength - 1] = 0x0D;
    xbee_write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
void xbee_setRegister_byte(char *parameter, unsigned char value)
{
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = strlen(parameter) + 4;
    unsigned char data[dataLength];
    int i;
    data[0] = 'A';
    data[1] = 'T';
    for (i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    data[param_size + 2] = value;
    data[dataLength - 1] = 0x0D;
    xbee_write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
void xbee_getRegister(char *parameter)
{
    unsigned char param_size = strlen(parameter);
    unsigned short dataLength = param_size + 3;
    unsigned char data[dataLength];
    int i;
    data[0] = 'A';
    data[1] = 'T';
    for (i=0; i<param_size; i++){
        data[i + 2] = parameter[i];
    }
    data[dataLength - 1] = 0x0D;
    xbee_write(data, dataLength);

    //attendre de recevoir la réponse du Xbee : OK\r
    //appeler decodeInit
}

// ____________________________________________________________
/*! \brief Called each time a packet of data is received from Xbee during init.
 *
 * \param buff_data: the buffer of bytes received.
 * \param buff_size: buffer size
  */
tXbeeErr xbee_decodeInit(unsigned char *buff_data, unsigned char buff_size)
{
    tXbeeErr dummy;
    if(buff_data[0] == 'O' && buff_data[1] == 'K' && buff_data[2] == '\r'){
        dummy = XBEE_OK;
    }
    else{
        dummy = XBEE_INIT_ERROR;
    }
    return dummy;
}


// ____________________________________________________________
tXbeeErr xbee_connect()
{
    tXbeeErr dummy=XBEE_OK;
    return dummy;
}

// ____________________________________________________________
unsigned char xbee_isConnected()
{
    unsigned char dummy = 1;
    return (dummy);
}

// ____________________________________________________________
tXbeeErr xbee_reset()
{
    tXbeeErr dummy=XBEE_OK;
    return dummy;
}

// ____________________________________________________________
/*! \brief Entry point to encode and send a buffer in Xbee format.
 *
 * \param buff_data: the buffer to encode.
 * \param buff_size: buffer size.
 * \param dest_address: Xbee destination ID.
 */
void xbee_encode(unsigned char *buff_data, unsigned short buff_size, unsigned short dest_address)
{

    unsigned char xbuff_size = buff_size + 9;
    unsigned char xbuff[xbuff_size];
    int i;
    xbuff[0] = 0x7E;                            //entete Xbee en mode API
    xbuff[1] = 0x00;                            //MSB longueur message : donnee utile + des parametres d'envoi du message
    xbuff[2] = buff_size + 5;                   //LSB longueur message : donnee utile + des parametres d'envoi du message
    xbuff[3] = 0x01;                            //definit fonction du message, ici envoi vers un autre module
    xbuff[4] = 0x00;                            //ACK : 0 = no ACK (pas tout compris)
    xbuff[5] = dest_address>>8;                 //MSB : dest_adress
    xbuff[6] = dest_address;                    //LSB : dest_adress
    xbuff[7] = 0x01;                            //OPTION : 1 = disable ACK
    for(i =0;i<buff_size;i++){              //donnee utile
        xbuff[8+i] = buff_data[i];
    }
    xbuff[buff_size+8] = xbee_getChecksum(xbuff);

    xbee_write(xbuff, xbuff_size);                   //envoi du message
}


// ____________________________________________________________
/*! \brief Called each time a packet of data is received from Xbee.
 *
 * \param buff_data: the buffer of bytes received.
 * \param buff_size: buffer size
  */
void xbee_decode_buff(unsigned char *buff_data, unsigned char buff_size)
{
    int i;
    for (i=0; i<buff_size; i++)
    {
        xbee_decode(buff_data[i]);
    }
}

// ____________________________________________________________
/*! \brief Called each time a byte is received from Xbee.
 *
 * \param newData: the byte received.
 * \remarks : build a XMessage step by step
  */
void xbee_decode(unsigned char newData)
{
    switch(m_xmessage_state) {
        //Réception d'une trame en mode API
        case XBEE_HEADER :      //Header du message Xbee
            if (newData == 0x7E) {
                m_xmessage_state = XBEE_LENGTH_MSB;
                m_current_xmessage.Checksum = 0;
                m_xmessage_index = 0;
            }
            break;
        case XBEE_LENGTH_MSB :      //Taille de la donnée + les options
            m_current_xmessage.DLC = newData << 8;
            m_xmessage_state = XBEE_LENGTH_LSB;
            break;
        case XBEE_LENGTH_LSB :      //Taille de la donnée + les options
            m_current_xmessage.DLC += newData;
            m_xmessage_state = XBEE_FRAME;
            break;
        case XBEE_FRAME :       //Type de message : réception d'un message = 0x81
            m_current_xmessage.FrameType = newData;
            if (m_current_xmessage.FrameType == 0x81){      //Reception d'un message Xbee
                m_current_xmessage.DLC -= 5;                //Taille de la donnée utile
                m_xmessage_state = XBEE_SOURCE_MSB;
            }
            else {
               m_xmessage_state = XBEE_HEADER;
            }
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_SOURCE_MSB :      //Source du message
            m_current_xmessage.SourceID = newData << 8;
            m_xmessage_state = XBEE_SOURCE_LSB;
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_SOURCE_LSB :      //Source du message
            m_current_xmessage.SourceID += newData;
            m_xmessage_state = XBEE_RSSI;
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_RSSI :        //Puissance du signal reçu
            m_xmessage_state = XBEE_OPTION;
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_OPTION :
            m_xmessage_state = XBEE_DATA;
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_DATA :
            m_current_xmessage.Data[m_xmessage_index] = newData;
            m_xmessage_index++;
            if (m_xmessage_index == m_current_xmessage.DLC){    //La donnée utile est récupérée
                m_xmessage_index = 0;
                m_xmessage_state = XBEE_CHECKSUM;
            }
            m_current_xmessage.Checksum += newData;
            break;
        case XBEE_CHECKSUM :                                    //Checksum du message Xbee : Vérifie la transmission UART
            if (newData == (0xFF - m_current_xmessage.Checksum)) {
                xbee_readyBytes_callback(m_current_xmessage.Data, m_current_xmessage.DLC, m_current_xmessage.SourceID);
            }
            m_xmessage_state = XBEE_HEADER;
            break;
        default :
            m_xmessage_state = XBEE_HEADER;
            break;
    }
}


// ____________________________________________________________
/*! \brief Compute checksum from the packet.
 *
 * \param packet : the packet to compute
 * \return the checksum
 * \remarks :
  */
unsigned char xbee_getChecksum(unsigned char *xpacket)
{
    unsigned char sum=0;
    unsigned short xpacket_size = ((unsigned short)xpacket[1]<<8) + xpacket[2];
    int i;
    for (i=3; i<3+xpacket_size; i++) {
        sum+= xpacket[i];
    }
    return ~sum;
}
