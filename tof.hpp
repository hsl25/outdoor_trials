#ifndef TOF_HPP
#define TOF_HPP

// I2C defines 
#define I2C_PORT i2c0
#define SDA_PIN 12
#define SCL_PIN 13
#define I2C_BAUDRATE 100000

// TX/RX defines
#define UART_TX_PIN 16
#define UART_RX_PIN 17
#define UART_ID uart0
#define UART_BAUD_RATE 115200

class TOF {
    public:
        TOF();
        void init_i2c();
        void init_uart();
        void setup_tof();
        void read_tof();
    private:
};

#endif

