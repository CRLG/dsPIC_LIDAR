#include "mcp23017.h"
#include "i2c_masterdrv.h"

static unsigned char m_mcp_address_8bits;
static unsigned char m_port_A;
static unsigned char m_port_B;

void mcp23017_init(unsigned char address_8bits)
{
  m_mcp_address_8bits = address_8bits;
  m_port_A = 0;
  m_port_B = 0;
}

// ____________________________________________________________
unsigned char mcp23017_readRegister(unsigned char reg_addr)
{
    unsigned char buff[1];
    i2c_master_read_register(m_mcp_address_8bits, reg_addr, buff, 1);
    return (buff[0]);
}

// ____________________________________________________________
void mcp23017_writeRegister(unsigned char reg_addr, unsigned char data)
{
    i2c_master_write_register(m_mcp_address_8bits, reg_addr, &data, 1);
}

// ____________________________________________________________
void mcp23017_configDirectionPortA(unsigned char direction)
{
    mcp23017_writeRegister(MCP23017_IODIRA, direction);
}

// ____________________________________________________________
void mcp23017_configDirectionPortB(unsigned char direction)
{
    mcp23017_writeRegister(MCP23017_IODIRB, direction);
}

// ____________________________________________________________
void mcp23017_configDirections(unsigned char directionA, unsigned char directionB)
{
    mcp23017_configDirectionPortA(directionA);
    mcp23017_configDirectionPortB(directionB);
}

// ____________________________________________________________
void mcp23017_configPullUpPortA(unsigned char pullup)
{
    mcp23017_writeRegister(MCP23017_GPPUA, pullup);
}

// ____________________________________________________________
void mcp23017_configPullUpPortB(unsigned char pullup)
{
    mcp23017_writeRegister(MCP23017_GPPUB, pullup);
}

// ____________________________________________________________
void mcp23017_configPullUp(unsigned char pullupA, unsigned char pullupB)
{
    mcp23017_configPullUpPortA(pullupA);
    mcp23017_configPullUpPortB(pullupB);
}

// ____________________________________________________________
void mcp23017_writePortA(unsigned char port_val)
{
    m_port_A = port_val;
    mcp23017_writeRegister(MCP23017_OLATA, port_val);
}

// ____________________________________________________________
void mcp23017_writePortB(unsigned char port_val)
{
    m_port_B = port_val;
    mcp23017_writeRegister(MCP23017_OLATB, port_val);
}

// ____________________________________________________________
void mcp23017_writePorts(unsigned char portA, unsigned char portB)
{
    mcp23017_writePortA(portA);
    mcp23017_writePortB(portB);
}

// ____________________________________________________________
void mcp23017_writeBitPortA(unsigned char bit_pos, unsigned char val)
{
    if (val) {
        m_port_A |= (1<<bit_pos);
    }
    else {
        m_port_A &= ~(1<<bit_pos);
    }
    mcp23017_writePortA(m_port_A);
}

// ____________________________________________________________
void mcp23017_writeBitPortB(unsigned char bit_pos, unsigned char val)
{
    if (val) {
        m_port_B |= (1<<bit_pos);
    }
    else {
        m_port_B &= ~(1<<bit_pos);
    }
    mcp23017_writePortB(m_port_B);
}


// ____________________________________________________________
unsigned char mcp23017_readPortA(void)
{
    return mcp23017_readRegister(MCP23017_GPIOA);
}

// ____________________________________________________________
unsigned char mcp23017_readPortB(void)
{
    return mcp23017_readRegister(MCP23017_GPIOB);
}

// ____________________________________________________________
void mcp23017_refreshOutputs(void)
{
    mcp23017_writePorts(m_port_A, m_port_B);
}

