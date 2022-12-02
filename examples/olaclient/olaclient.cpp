#include <ola/DmxBuffer.h>    // sudo apt-get install libola-dev
#include <ola/Logging.h>
#include <ola/OlaClientWrapper.h>
#include <string>
#include <bitset>
#include <ctime>
using namespace std;

#include <PCA9685.h>
#include <iostream>
#include "config.h"
#include "../../src/PCA9685.h"

#define PWM_FREQ 200
#define DMX_UNIVERSE 1
#define I2C_ADPT 1
#define I2C_ADDR 0x40

#define BYTES_PER_I2C  (_PCA9685_CHANS * 2)

// global var for i2c file descriptor
int i2c_fd;


// Called when universe registration completes.
void RegisterComplete(const ola::client::Result& result) {
  if (!result.Success()) {
    OLA_WARN << "Failed to register universe: " << result.Error();
  } // if err
}


// Called when new DMX data arrives.
void NewDmx(const ola::client::DMXMetadata &metadata,
            const ola::DmxBuffer &data) {

    const int NUM_I2C_ADDRS = 3;
    unsigned int universe = metadata.universe;

  static string inData="";
  inData = data.Get();
  unsigned int onVals[_PCA9685_CHANS * NUM_I2C_ADDRS];
  static unsigned int offVals[_PCA9685_CHANS * NUM_I2C_ADDRS];
  for (int i = 0; i < _PCA9685_CHANS; i++) {
        onVals[i] = 0;
  } // for

  int dmxVals[_PCA9685_CHANS];

  unsigned char i2cAddrs[NUM_I2C_ADDRS] = {0x40, 0x43, 0x44};
 
  // 16-bit ola values so two 8-bit dmx channels per 12-bit pwm value
  for (unsigned int dmxChan = 0; dmxChan < inData.length(); dmxChan++) {

    // disregard any data after pwm values
    // TODO: disregard any data after declared I2C addrs
    if (dmxChan >= BYTES_PER_I2C * NUM_I2C_ADDRS) break;
 
    // get both bytes and increment dmxChan
    int msb = inData.at(dmxChan);
    dmxChan++;
    int lsb = inData.at(dmxChan);
    int pwmChan = dmxChan / 2;
    dmxVals[pwmChan] = msb * 256 + lsb;
 
    // convert 16-bit to 12-bit
    unsigned int pwmVal = dmxVals[pwmChan] >> 4;
 
    if (pwmVal != offVals[pwmChan]) {
      //cout << showbase << std::internal << setfill('0');
      //cout << dec << setw(2) << pwmChan << ": ";
      //cout << hex << setw(5) << pwmVal << endl;
    } // if

    // store the 12-bit pwmVal
    offVals[pwmChan] = pwmVal;

//    int i2cAddrIdx = dmxChan >= _PCA9685_CHANS * 2
    // update all PWM values once at end of frame
    if ((dmxChan + 1) % BYTES_PER_I2C == 0) {

        unsigned int onValsCache[_PCA9685_CHANS];
        static unsigned int offValsCache[_PCA9685_CHANS];
        // manually deep copy elements
        for (int i = 0; i < _PCA9685_CHANS; i++){
            onValsCache[i] = onVals[dmxChan];
            offValsCache[i] = offVals[dmxChan];
        }


      // update all channels from offVals
      int ret;
      ret = PCA9685_setPWMVals(i2c_fd, i2cAddrs[dmxChan/BYTES_PER_I2C], onValsCache, offValsCache);
      if (ret != 0) {
        cout << "NewDMX(): PCA9685_setPWMVals() returned " << ret << endl;
        return;
      } // if err
    } // if chan
  } // for chan
} // NewDMX


int main() {
  cout << "olaclient " << libPCA9685_VERSION_MAJOR << "." << libPCA9685_VERSION_MINOR << endl;
  cout << "ohai" << endl;
  // setup I2C device
  i2c_fd = PCA9685_openI2C(I2C_ADPT, I2C_ADDR);
  if (i2c_fd < 0) {
    cout << "main(): PCA9685_openI2C() returned " << i2c_fd;
    cout << " on I2C_ADPT " << I2C_ADPT << " at I2C_ADDR " << I2C_ADDR << endl;
    return i2c_fd;
  } // if err

  // setup PCA9685 device
  int ret;
  ret = PCA9685_initPWM(i2c_fd, I2C_ADDR, PWM_FREQ);
  if (ret != 0) {
    cout << "main(): PCA9685_initPWM() returned " << ret;
    cout << " I2C_ADDR " << I2C_ADDR << " PWM_FREQ " << PWM_FREQ << endl;
    return ret;
  } // if err

  // setup ola logging and wrapper
  ola::InitLogging(ola::OLA_LOG_INFO, ola::OLA_LOG_STDERR);
  ola::client::OlaClientWrapper wrapper;
  ret = wrapper.Setup();
  if (ret == 0) {
    cout << "main(): OlaClientWrapper.Setup() returned 0" << endl;
    return 1;
  } // if err

  // connect ola to client
  ola::client::OlaClient *client = wrapper.GetClient();
  // Set the callback and register our interest in this universe
  client->SetDMXCallback(ola::NewCallback(&NewDmx));

  for (int i = 0; i < 5; i++) {
      client->RegisterUniverse(
              DMX_UNIVERSE + i, ola::client::REGISTER, ola::NewSingleCallback(&RegisterComplete));
      wrapper.GetSelectServer()->Run();
  }
}

