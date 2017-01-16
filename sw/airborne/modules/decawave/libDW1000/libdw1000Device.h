#ifndef _DW1000Device_H_INCLUDED
#define _DW1000Device_H_INCLUDED

#define INACTIVITY_TIME 1000
#include "libdw1000Types.h"
#include "mbed.h"
#include "math.h"



class dwDistanceDevice {
    public:
        //Constructor and destructor
        dwDistanceDevice(); 
        dwDistanceDevice(uint8_t address[]);
        ~dwDistanceDevice();
    
        //setters:
        void setReplyTime(unsigned int replyDelayTimeUs);
        void setAddress(uint8_t address[]);
    
        void setRange(float range);
        void setRXPower(float power);
        void setFPPower(float power);
        void setQuality(float quality);
    
        void setReplyDelayTime(int time){ _replyDelayTimeUS=time;}
    
        void setIndex(short index){ _index=index; }
    
        //getters
        unsigned int getReplyTime(){ return _replyDelayTimeUS; }
        uint8_t* getddress();
        short getIndex(){return _index;}
    
        //String getAddress();
        uint8_t* getAddress();
    
        //String getShortAddress();
    
        float getRange();
        float getRXPower();
        float getFPPower();
        float getQuality();
    
        bool isAddressEqual(dwDistanceDevice *device);

    
        //functions which contains the date: (easier to put as public)
        // timestamps to remember
        dwTime_t poll_tx;
        dwTime_t poll_rx;
        dwTime_t answer_tx;
        dwTime_t answer_rx;
        dwTime_t final_tx;
        dwTime_t final_rx;
        dwTime_t report_rx;
        
        volatile bool _ranging_finished;
    
       // void noteActivity();
       // bool isInactive();
    
    
    private:
        //device ID
        uint8_t _address[8];
        long _activity;
        unsigned int _replyDelayTimeUS;
        short _index;
    
        int _range;
        int _RXPower;
        int _FPPower;
        int _quality;
    
        void randomAddress();
 
};
#endif