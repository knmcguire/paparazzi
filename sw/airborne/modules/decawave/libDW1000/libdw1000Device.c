#include "libdw1000Device.h"



//Constructor and destructor
dwDistanceDevice::dwDistanceDevice(): _ranging_finished(false){
    randomAddress();
}

dwDistanceDevice::dwDistanceDevice(uint8_t deviceAddress[]):_ranging_finished(false){

    //we have a short address (2 bytes)
    setAddress(deviceAddress);
}

dwDistanceDevice::~dwDistanceDevice(){
}

//setters:
void dwDistanceDevice::setReplyTime(unsigned int replyDelayTimeUs){ _replyDelayTimeUS=replyDelayTimeUs; }
//void dwDistanceDevice::setAddress(char deviceAddress[]){ DW1000.convertToByte(deviceAddress, _address); }

void dwDistanceDevice::setAddress(uint8_t deviceAddress[]){
    memcpy(_address, deviceAddress, 8);
}



void dwDistanceDevice::setRange(float range){ _range=ceil(range*100);}
void dwDistanceDevice::setRXPower(float RXPower){ _RXPower=ceil(RXPower*100); }
void dwDistanceDevice::setFPPower(float FPPower){ _FPPower=ceil(FPPower*100); }
void dwDistanceDevice::setQuality(float quality){ _quality=ceil(quality*100); }





uint8_t* dwDistanceDevice::getAddress(){
    return _address;
}
/*
String DW1000Device::getAddress(){
    char string[25];
    sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            _ownAddress[0], _ownAddress[1], _ownAddress[2], _ownAddress[3], _ownAddress[4], _ownAddress[5], _ownAddress[6], _ownAddress[7]);
    return String(string);
}*/



/*
String DW1000Device::getShortAddress(){
    char string[6];
    sprintf(string, "%02X:%02X",
            _shortAddress[0], _shortAddress[1]);
    return String(string);
}
*/


bool dwDistanceDevice::isAddressEqual(dwDistanceDevice *device){
    if(memcmp(this->getAddress(),device->getAddress(), 8)==0)
    {
        return true;
    }
    else{
        return false;
    }
}


float dwDistanceDevice::getRange(){ return float(_range)/100.0f; }
float dwDistanceDevice::getRXPower(){ return float(_RXPower)/100.0f; }
float dwDistanceDevice::getFPPower(){ return float(_FPPower)/100.0f; }
float dwDistanceDevice::getQuality(){ return float(_quality)/100.0f; }

void dwDistanceDevice::randomAddress(){
    _address[0]=ceil(rand() * 256.0);
    _address[1]=ceil(rand() * 256.0);
    _address[2]=ceil(rand() * 256.0);
    _address[3]=ceil(rand() * 256.0);
    _address[4]=ceil(rand() * 256.0);
    _address[5]=ceil(rand() * 256.0);
    _address[6]=ceil(rand() * 256.0);
    _address[7]=ceil(rand() * 256.0);
}

/*
void dwDistanceDevice::noteActivity(){
    _activity=millis();
}

bool dwDistanceDevice::isInactive(){
    //One second of inactivity
    if(millis()-_activity>INACTIVITY_TIME){
        _activity=millis();
        return true;
    }
    return false;
}*/