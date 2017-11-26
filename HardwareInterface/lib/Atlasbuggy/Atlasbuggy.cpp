
#include <Atlasbuggy.h>
// #define DEBUG

Atlasbuggy::Atlasbuggy(String whoiam)
{
    _command = "";
    _whoiam = whoiam;
    _initPacket = "";
    _paused = true;
    _led13_state = false;
}

void Atlasbuggy::begin()
{
    Serial.begin(DEFAULT_RATE);
    Serial.setTimeout(100);  // 100 ms
    pinMode(LED13, OUTPUT);
    #ifdef DEBUG
    Serial.print("BAUD is ");
    Serial.println(DEFAULT_RATE);
    #endif

    setLed(true);
}

void Atlasbuggy::setInitData(String initData) {
    _initPacket = initData;
}

bool Atlasbuggy::available() {
    return Serial.available() > 0;
}

int Atlasbuggy::readSerial()
{
    if (_paused) {
        delay(100);  // minimize activity while paused
    }
    _command = Serial.readStringUntil('\n');
    #ifdef DEBUG
    Serial.println(_command);
    #endif

    if (_command.equals("whoareyou")) {
        writeWhoiam();
        return 4;
    }
    else if (_command.equals("init?")) {
        writeInit();
        return 3;
    }
    else if (_command.equals("start")) {
        if (unpause()) return 2;
        else return -1;
    }
    else if (_command.equals("stop")) {
        if (pause()) return 1;
        else return -1;
    }
    else if (_command.substring(0, 1).equals("s")) {
        return 5;
    }
    else {
        if (!_paused) return 0;
        else return -1;
    }
}

void Atlasbuggy::changeBaud(int newBaud)
{
    #ifdef DEBUG
    Serial.println("changing baud");
    #endif
    delay(50);
    Serial.end();
    delay(50);
    Serial.begin(newBaud);
}

void Atlasbuggy::setLed(bool state)
{
    _led13_state = state;
    digitalWrite(LED13, state);
}

bool Atlasbuggy::getLed() {
    return _led13_state;
}

String Atlasbuggy::getCommand() {
    return _command;
}

bool Atlasbuggy::isPaused() {
    return _paused;
}



void Atlasbuggy::writeWhoiam()
{
    Serial.print("iam");
    Serial.print(_whoiam);
    Serial.print(PACKET_END);
}

void Atlasbuggy::writeInit()
{
    Serial.print("init:");
    Serial.print(_initPacket);
    Serial.print(PACKET_END);
}

bool Atlasbuggy::unpause()
{
    #ifdef DEBUG
    Serial.print("_paused is ");
    Serial.println(_paused);
    #endif
    if (_paused) {
        setLed(LOW);
        _paused = false;
        return true;
    }
    else {
        return false;
    }
}

bool Atlasbuggy::pause()
{
    if (!_paused) {
        Serial.print("\nstopping\n");
        setLed(HIGH);
        _paused = true;
        return true;
    }
    else {
        return false;
    }
}
