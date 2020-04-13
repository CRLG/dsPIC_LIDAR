#ifndef _MCP23017_H_
#define _MCP23017_H_

typedef enum {
     MCP23017_IODIRA      = 0x00,
     MCP23017_IODIRB      = 0x01,
     MCP23017_IPOLA       = 0x02,
     MCP23017_IPOLB       = 0x03,
     MCP23017_GPINTENA    = 0x04, 
     MCP23017_GPINTENB    = 0x05,
     MCP23017_GPPUA       = 0x0C,
     MCP23017_GPPUB       = 0x0D,
     MCP23017_GPIOA       = 0x12,
     MCP23017_GPIOB       = 0x13,
     MCP23017_OLATA       = 0x14,
     MCP23017_OLATB       = 0x15
}tRegMCP23017;

    void mcp23017_init(unsigned char address_8bits);
    void mcp23017_setAddress(unsigned char address_8bits);

    unsigned char mcp23017_readRegister(unsigned char reg_addr);
    void mcp23017_writeRegister(unsigned char reg_addr, unsigned char data);

    void mcp23017_configDirectionPortA(unsigned char direction);
    void mcp23017_configDirectionPortB(unsigned char direction);
    void mcp23017_configDirections(unsigned char directionA, unsigned char directionB);

    void mcp23017_configPullUpPortA(unsigned char pullup);
    void mcp23017_configPullUpPortB(unsigned char pullup);
    void mcp23017_configPullUp(unsigned char pullupA, unsigned char pullupB);

    void mcp23017_writePortA(unsigned char port_val);
    void mcp23017_writePortB(unsigned char port_val);
    void mcp23017_writePorts(unsigned char portA, unsigned char portB);
    void mcp23017_writeBitPortA(unsigned char bit_pos, unsigned char val);
    void mcp23017_writeBitPortB(unsigned char bit_pos, unsigned char val);

    unsigned char mcp23017_readPortA(void);
    unsigned char mcp23017_readPortB(void);


#endif // _MCP23017_H_
