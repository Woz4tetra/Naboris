#include <Arduino.h>

#define DEFAULT_RATE 115200
#define LED13 13
#define PACKET_END '\n'

class Atlasbuggy {
public:
    Atlasbuggy(String whoiam);
    void begin();
    void setInitData(String initData);
    int readSerial();
    String getCommand();
    bool isPaused();
    void setLed(bool state);
    bool getLed();

    void changeBaud(int newBaud);
    bool available();

    bool unpause();
    bool pause();

private:
    String _command;
    String _whoiam;
    String _initPacket;
    bool _paused;
    bool _led13_state;

    void writeWhoiam();
    void writeInit();
};
